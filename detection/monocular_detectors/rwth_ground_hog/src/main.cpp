// ROS includes.
#include <ros/ros.h>

#if WITH_CUDA
#include <ros/time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <string.h>
#include <QImage>
#include <QPainter>


#include <cudaHOG.h>

#include <rwth_perception_people_msgs/GroundHOGDetections.h>
#include <rwth_perception_people_msgs/GroundPlane.h>

#include <spencer_tracking_msgs/DetectedPersons.h>
#include <spencer_diagnostics/publisher.h>

#include "Matrix.h"
#include "Vector.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace rwth_perception_people_msgs;


cudaHOG::cudaHOGManager *hog;
ros::Publisher pub_message;
image_transport::Publisher pub_result_image;
spencer_diagnostics::MonitoredPublisher pub_detected_persons;
double worldScale; // for computing 3D positions from BBoxes
double score_thresh; // threshold for HOG detections

int detection_id_increment, detection_id_offset, current_detection_id; // added for multi-sensor use in SPENCER
double pose_variance; // used in output spencer_tracking_msgs::DetectedPerson.pose.covariance


void render_bbox_2D(GroundHOGDetections& detections, QImage& image, int r, int g, int b, int lineWidth)
{

    QPainter painter(&image);
    QColor qColor;
    QPen pen;

    for(int i = 0; i < detections.pos_x.size(); i++){
        int x =(int) detections.pos_x[i];
        int y =(int) detections.pos_y[i];
        int w =(int) detections.width[i];
        int h =(int) detections.height[i];
        float score = detections.score[i];

        qColor.setRgb(min(255,(int)(score*100)), 0, 0);
        pen.setColor(qColor);
        pen.setWidth(lineWidth+(int)score);
        painter.setPen(pen);
        pen.setColor(qColor);

        painter.drawLine(x,y, x+w,y);
        painter.drawLine(x,y, x,y+h);
        painter.drawLine(x+w,y, x+w,y+h);
        painter.drawLine(x,y+h, x+w,y+h);
    }
}

// NOTE: Not used in SPENCER! We use the version with ground plane below!
void imageCallback(const Image::ConstPtr &msg)
{
    //    ROS_INFO("Entered img callback");
    std::vector<cudaHOG::Detection> detHog;

    //  unsigned char image
    QImage image_rgb(&msg->data[0], msg->width, msg->height, QImage::Format_RGB888);
    int returnPrepare = hog->prepare_image(image_rgb.convertToFormat(QImage::Format_ARGB32).bits(), (short unsigned int) msg->width, (short unsigned int) 	msg->height);

    if(returnPrepare)
    {
        ROS_ERROR("groundHOG: Error while preparing the image");
        return;
    }

    hog->test_image(detHog);
    hog->release_image();

    int w = 64, h = 128;

    GroundHOGDetections detections;

    detections.header = msg->header;
    for(unsigned int i=0;i<detHog.size();i++)
    {
        float score = detHog[i].score;
        float scale = detHog[i].scale;

        float width = (w - 32.0f)*scale;
        float height = (h - 32.0f)*scale;
        float x = (detHog[i].x + 16.0f*scale);
        float y = (detHog[i].y + 16.0f*scale);

        detections.scale.push_back(scale);
        detections.score.push_back(score);
        detections.pos_x.push_back(x);
        detections.pos_y.push_back(y);
        detections.width.push_back(width);
        detections.height.push_back(height);

    }

    if(pub_result_image.getNumSubscribers()) {
        ROS_DEBUG("Publishing image");
        render_bbox_2D(detections, image_rgb, 255, 0, 0, 2);

        Image sensor_image;
        sensor_image.header = msg->header;
        sensor_image.height = image_rgb.height();
        sensor_image.width  = image_rgb.width();
        sensor_image.step   = msg->step;
        vector<unsigned char> image_bits(image_rgb.bits(), image_rgb.bits()+sensor_image.height*sensor_image.width*3);
        sensor_image.data = image_bits;
        sensor_image.encoding = msg->encoding;

        pub_result_image.publish(sensor_image);
    }

    pub_message.publish(detections);
}

void getRay(const Matrix<double>& K, const Vector<double>& x, Vector<double>& ray1, Vector<double>& ray2)
{
    Matrix<double> Kinv = K;
    Kinv.inv();

    ray1 = Vector<double>(3, 0.0);

    Matrix<double> rot = Eye<double>(3);
    rot *= Kinv;
    ray2 = rot * x;
    //ray2 += ray1;
}

void intersectPlane(const Vector<double>& gp, double gpd, const Vector<double>& ray1, const Vector<double>& ray2, Vector<double>& point)
{
    Vector<double> diffRay;
    diffRay = ray1;
    diffRay -= ray2;

    double den = DotProduct(gp, diffRay);
    double t = (DotProduct(gp, ray1) + gpd) / den;

    point = ray1;
    diffRay = (ray2);
    diffRay -= (ray1);
    diffRay *= t;
    point += diffRay;
}

void calc3DPosFromBBox(const Matrix<double>& K, const Vector<double>& GPN_, double GPD_, double x, double y, double w, double h, double ConvertScale, Vector<double>& pos3D)
{
    // bottom_center is point of the BBOX
    Vector<double> bottom_center(3, 1.0);
    bottom_center(0) = x + w/2.0;
    bottom_center(1) = y + h;

    // Backproject through base point
    Vector<double> ray_bot_center_1;
    Vector<double> ray_bot_center_2;
    getRay(K, bottom_center, ray_bot_center_1, ray_bot_center_2);
    
    // Intersect with ground plane
    Vector<double> gpPointCenter;
    intersectPlane(GPN_, GPD_, ray_bot_center_1, ray_bot_center_2, gpPointCenter);
   
    // Compute 3D Position of BBOx
    double posX = gpPointCenter(0) * ConvertScale;
    double posY = gpPointCenter(1) * ConvertScale;
    double posZ = gpPointCenter(2) * ConvertScale;

    pos3D.setSize(3);
    pos3D(0) = posX;
    pos3D(1) = posY;
    pos3D(2) = posZ;
}


void imageGroundPlaneCallback(const ImageConstPtr &color, const CameraInfoConstPtr &camera_info,
                              const GroundPlaneConstPtr &gp)
{
    //    ROS_INFO("Entered gp-img callback");
    std::vector<cudaHOG::Detection> detHog;

    //  unsigned char image
    QImage image_rgb(&color->data[0], color->width, color->height, QImage::Format_RGB888);
    int returnPrepare = hog->prepare_image(image_rgb.convertToFormat(QImage::Format_ARGB32).bits(),
                                           (short unsigned int) color->width, (short unsigned int) color->height);

    if(returnPrepare)
    {
        ROS_ERROR("Error by preparing the image");
        return;
    }

    // Generate base camera
    Matrix<float> R = Eye<float>(3);
    Vector<float> t(3, 0.0);

    // Get GP
    Vector<double> GPN(3, (double*) &gp->n[0]);
    double GPd = ((double) gp->d)*(-1000.0); // GPd = -958.475;
    Matrix<double> K(3,3, (double*)&camera_info->K[0]);

    // NOTE: Using 0 1 0 does not work, apparently due to numerical problems in libCudaHOG (E(1,1) gets zero when solving quadratic form)
    Vector<float> float_GPN(3);
    float_GPN(0) = -0.0123896; //-float(GPN(0));
    float_GPN(1) = 0.999417; //-float(GPN(1)); // swapped with z by Timm
    float_GPN(2) = 0.0317988; //-float(GPN(2));

    float float_GPd = (float) GPd;
    Matrix<float> float_K(3,3);
    float_K(0,0) = K(0,0); float_K(1,0) = K(1,0); float_K(2,0) = K(2,0);
    float_K(1,1) = K(1,1); float_K(0,1) = K(0,1); float_K(2,1) = K(2,1);
    float_K(2,2) = K(2,2); float_K(0,2) = K(0,2); float_K(1,2) = K(1,2);

    //ROS_WARN("Ground plane: %.2f %.2f %.2f d=%.3f", float_GPN(0), float_GPN(1), float_GPN(2), float_GPd);

    // If assertion fails, probably ground plane is wrongly oriented!?
    hog->set_camera(R.data(), float_K.data(), t.data());
    hog->set_groundplane(float_GPN.data(), &float_GPd);
    hog->prepare_roi_by_groundplane();
    hog->test_image(detHog);
    hog->release_image();

    const int WINDOW_WIDTH = 64, WINDOW_HEIGHT = 128;

    GroundHOGDetections detections;

    detections.header = color->header;
    for(unsigned int i=0;i<detHog.size();i++)
    {

        float score = detHog[i].score;
        if (score < score_thresh) continue;
        float scale = detHog[i].scale;

        float width = (WINDOW_WIDTH - 32.0f)*scale;
        float height = (WINDOW_HEIGHT - 32.0f)*scale;
        float x = (detHog[i].x + 16.0f*scale);
        float y = (detHog[i].y + 16.0f*scale);

        detections.scale.push_back(scale);
        detections.score.push_back(score);
        detections.pos_x.push_back(x);
        detections.pos_y.push_back(y);
        detections.width.push_back(width);
        detections.height.push_back(height);

    }

    if(pub_result_image.getNumSubscribers()) {
        ROS_DEBUG("Publishing image");
        render_bbox_2D(detections, image_rgb, 255, 0, 0, 2);

        Image sensor_image;
        sensor_image.header = color->header;
        sensor_image.height = image_rgb.height();
        sensor_image.width  = image_rgb.width();
        sensor_image.step   = color->step;
        vector<unsigned char> image_bits(image_rgb.bits(), image_rgb.bits()+sensor_image.height*sensor_image.width*3);
        sensor_image.data = image_bits;
        sensor_image.encoding = color->encoding;

        pub_result_image.publish(sensor_image);
    }

    pub_message.publish(detections);


    //
    // Now create 3D coordinates for SPENCER DetectedPersons msg
    //
    if(pub_detected_persons.getNumSubscribers()) {
        spencer_tracking_msgs::DetectedPersons detected_persons;
        detected_persons.header = color->header;

        for(unsigned int i=0;i<detHog.size();i++)
        {
            float score = detHog[i].score;
            if (score < score_thresh) continue;
            float scale = detHog[i].scale;

            // FIXME: Is it correct to use these offsets for computing 3D position!?
            float width = (WINDOW_WIDTH - 32.0f)*scale;
            float height = (WINDOW_HEIGHT - 32.0f)*scale;
            float x = (detHog[i].x + 16.0f*scale);
            float y = (detHog[i].y + 16.0f*scale);

            Vector<double> normal(3, 0.0);
            normal(0) = GPN(0);
            normal(1) = GPN(1);
            normal(2) = GPN(2);

            Vector<double> pos3D;
            calc3DPosFromBBox(K, normal, GPd, x, y, width, height, worldScale, pos3D);       

            // DetectedPerson for SPENCER
            spencer_tracking_msgs::DetectedPerson detected_person;
            detected_person.modality = spencer_tracking_msgs::DetectedPerson::MODALITY_GENERIC_MONOCULAR_VISION;
            detected_person.confidence = detHog[i].score; // FIXME: normalize
            detected_person.pose.pose.position.x = -pos3D(0);
            detected_person.pose.pose.position.y = -pos3D(1);
            detected_person.pose.pose.position.z = -pos3D(2);  
            detected_person.pose.pose.orientation.w = 1.0;

            const double LARGE_VARIANCE = 999999999;
            detected_person.pose.covariance[0*6 + 0] = pose_variance;
            detected_person.pose.covariance[1*6 + 1] = pose_variance; // up axis (since this is in sensor frame!)
            detected_person.pose.covariance[2*6 + 2] = pose_variance;
            detected_person.pose.covariance[3*6 + 3] = LARGE_VARIANCE;
            detected_person.pose.covariance[4*6 + 4] = LARGE_VARIANCE;
            detected_person.pose.covariance[5*6 + 5] = LARGE_VARIANCE;

            detected_person.detection_id = current_detection_id;
            current_detection_id += detection_id_increment;

            detected_persons.detections.push_back(detected_person);  
        }

        // Publish
        pub_detected_persons.publish(detected_persons);
    }
}

// Connection callback that unsubscribes from the tracker if no one is subscribed.
void connectCallback(ros::Subscriber &sub_msg,
                     ros::NodeHandle &n,
                     string gp_topic,
                     string img_topic,
                     Subscriber<GroundPlane> &sub_gp,
                     Subscriber<CameraInfo> &sub_cam,
                     image_transport::SubscriberFilter &sub_col,
                     image_transport::ImageTransport &it){
    if(!pub_message.getNumSubscribers() && !pub_result_image.getNumSubscribers() && !pub_detected_persons.getNumSubscribers()) {
        ROS_DEBUG("HOG: No subscribers. Unsubscribing.");
        sub_msg.shutdown();
        sub_gp.unsubscribe();
        sub_cam.unsubscribe();
        sub_col.unsubscribe();
    } else {
        ROS_DEBUG("HOG: New subscribers. Subscribing.");
        if(strcmp(gp_topic.c_str(), "") == 0) {
            sub_msg = n.subscribe(img_topic.c_str(), 1, &imageCallback);
        }
        sub_cam.subscribe();
        sub_gp.subscribe();
        sub_col.subscribe(it,sub_col.getTopic().c_str(),1);
    }
}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "groundHOG");
    ros::NodeHandle n;

    // Declare variables that can be modified by launch file or command line.
    int queue_size;
    string ground_plane;
    string camera_ns;
    string pub_topic;
    string pub_image_topic;
    string pub_topic_detected_persons;
    string conf;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("queue_size", queue_size, int(10));
    private_node_handle_.param("model", conf, string(""));

    private_node_handle_.param("camera_namespace", camera_ns, string("/head_xtion"));
    private_node_handle_.param("ground_plane", ground_plane, string(""));
    private_node_handle_.param("score_thresh", score_thresh, 0.0);

    // For SPENCER DetectedPersons message
    private_node_handle_.param("world_scale", worldScale, 1.0); // default for ASUS sensors
    private_node_handle_.param("detection_id_increment", detection_id_increment, 1);
    private_node_handle_.param("detection_id_offset",    detection_id_offset, 0);
    private_node_handle_.param("pose_variance",    pose_variance, 0.05);
    current_detection_id = detection_id_offset;

    string image_color = camera_ns + "/rgb/image_rect_color";
    string camera_info = camera_ns + "/rgb/camera_info";


    //Initialise cudaHOG
    if(strcmp(conf.c_str(),"") == 0) {
        ROS_ERROR("No model path specified.");
        ROS_ERROR("Run with: rosrun rwth_ground_hog groundHOG _model:=/path/to/model");
        exit(0);
    }

    ROS_DEBUG("groundHOG: Queue size for synchronisation is set to: %i", queue_size);

    hog = new  cudaHOG::cudaHOGManager();
    hog->read_params_file(conf);
    hog->load_svm_models();

    // Image transport handle
    image_transport::ImageTransport it(private_node_handle_);

    // Create a subscriber.
    // Name the topic, message queue, callback function with class name, and object containing callback function.
    // Set queue size to 1 because generating a queue here will only pile up images and delay the output by the amount of queued images
    ros::Subscriber sub_message; //Subscribers have to be defined out of the if scope to have affect.
    Subscriber<GroundPlane> subscriber_ground_plane(n, ground_plane.c_str(), 1); subscriber_ground_plane.unsubscribe();
    image_transport::SubscriberFilter subscriber_color;
    subscriber_color.subscribe(it, image_color.c_str(), 1); subscriber_color.unsubscribe();
    Subscriber<CameraInfo> subscriber_camera_info(n, camera_info.c_str(), 1); subscriber_camera_info.unsubscribe();

    ros::SubscriberStatusCallback con_cb = boost::bind(&connectCallback,
                                                       boost::ref(sub_message),
                                                       boost::ref(n),
                                                       ground_plane,
                                                       image_color,
                                                       boost::ref(subscriber_ground_plane),
                                                       boost::ref(subscriber_camera_info),
                                                       boost::ref(subscriber_color),
                                                       boost::ref(it));

    image_transport::SubscriberStatusCallback image_cb = boost::bind(&connectCallback,
                                                                   boost::ref(sub_message),
                                                                   boost::ref(n),
                                                                   ground_plane,
                                                                   image_color,
                                                                   boost::ref(subscriber_ground_plane),
                                                                   boost::ref(subscriber_camera_info),
                                                                   boost::ref(subscriber_color),
                                                                   boost::ref(it));

    //The real queue size for synchronisation is set here.
    sync_policies::ApproximateTime<Image, CameraInfo, GroundPlane> MySyncPolicy(queue_size);
    MySyncPolicy.setAgePenalty(1000); //set high age penalty to publish older data faster even if it might not be correctly synchronized.

    const sync_policies::ApproximateTime<Image, CameraInfo, GroundPlane> MyConstSyncPolicy = MySyncPolicy;
    Synchronizer< sync_policies::ApproximateTime<Image, CameraInfo, GroundPlane> > sync(MyConstSyncPolicy,
                                                                                        subscriber_color,
                                                                                        subscriber_camera_info,
                                                                                        subscriber_ground_plane);

    // Decide which call back should be used.
    if(strcmp(ground_plane.c_str(), "") == 0) {
        sub_message = n.subscribe(image_color.c_str(), 1, &imageCallback);
    } else {
        sync.registerCallback(boost::bind(&imageGroundPlaneCallback, _1, _2, _3));
    }

    // Create publishers
    private_node_handle_.param("detections", pub_topic, string("/groundHOG/detections"));
    pub_message = n.advertise<rwth_perception_people_msgs::GroundHOGDetections>(pub_topic.c_str(), 10, con_cb, con_cb);

    private_node_handle_.param("result_image", pub_image_topic, string("/groundHOG/image"));
    pub_result_image = it.advertise(pub_image_topic.c_str(), 1, image_cb, image_cb);

    private_node_handle_.param("detected_persons", pub_topic_detected_persons, string("/detected_persons"));
    pub_detected_persons = n.advertise<spencer_tracking_msgs::DetectedPersons>(pub_topic_detected_persons, 10, con_cb, con_cb);

    double min_expected_frequency, max_expected_frequency;
    private_node_handle_.param("min_expected_frequency", min_expected_frequency, 8.0);
    private_node_handle_.param("max_expected_frequency", max_expected_frequency, 100.0);
    
    pub_detected_persons.setExpectedFrequency(min_expected_frequency, max_expected_frequency);
    pub_detected_persons.setMaximumTimestampOffset(0.3, 0.1);
    pub_detected_persons.finalizeSetup();

    ros::spin();

    return 0;
}

#else

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "groundHOG");
    ros::NodeHandle n;
    ROS_ERROR("rwth_ground_hog package has been compiled without libcudaHOG.");
    ROS_ERROR("Please see rwth_perception_people/3rd_party for instructions.");
    ROS_ERROR("This node will have no functionality unless compiled with cuda support.");
}

#endif

