#include "detector.h"
#include "KConnectedComponentLabeler.h"


Detector::Detector()
{
}

void Detector::ProcessFrame(const Camera &camera_origin, const Matrix<double> &depth_map, const PointCloud &point_cloud,
                            const Matrix<double> &upper_body_template, Vector<Vector<double> > &detected_bounding_boxes)
{
    int width = depth_map.x_size();
    int height = depth_map.y_size();

//    Matrix<int> labeledROIs;
    Matrix<int> mat_2D_pos_x(width, height, -1);
    Matrix<int> mat_2D_pos_y(width, height, -1);

    // Compute 2D_positions, and occ_Map matrices
    ComputeFreespace(camera_origin, labeledROIs, mat_2D_pos_x, mat_2D_pos_y, point_cloud);

    Vector<ROI> all_ROIs;
    PreprocessROIs(labeledROIs, all_ROIs);

//    Vector<Vector<Vector<double> > > points3D_in_ROIs(all_ROIs.getSize());
    ExtractPointsInROIs(/*points3D_in_ROIs*/all_ROIs, mat_2D_pos_x, mat_2D_pos_y, point_cloud, labeledROIs);

    double scaleZ = Globals::freespace_scaleZ;

    Vector<double> plane_in_camera = camera_origin.get_GP();


    Vector<Vector<double> > close_range_BBoxes;
    Vector<Vector<double > > distances;

    Vector<double> lower_point(3);

    for(int l = 0; l < all_ROIs.getSize(); l++)
    {
//        if(all_ROIs(l).getSize() > 0)
        if(all_ROIs(l).has_any_point)
        {
//            int min_y_index = 0;
//            AncillaryMethods::MinVecVecDim(points3D_in_ROIs(l), 1, min_y_index);
//            Vector<double> lower_point = points3D_in_ROIs(l)(min_y_index);
            lower_point(0) = point_cloud.X(all_ROIs(l).ind_min_y);
            lower_point(1) = point_cloud.Y(all_ROIs(l).ind_min_y);
            lower_point(2) = point_cloud.Z(all_ROIs(l).ind_min_y);

            double height = plane_in_camera(0)*lower_point(0) + plane_in_camera(1)*lower_point(1) +
                    plane_in_camera(2)*lower_point(2) + plane_in_camera(3)*Globals::WORLD_SCALE;
            ROS_DEBUG_STREAM("Checking height: " << Globals::min_height << " < " << height << " < " << Globals::max_height);
            ROS_DEBUG_STREAM("Checking distance: " << (all_ROIs(l).center_y()-all_ROIs(l).width()/2.0)/scaleZ << " < " << Globals::distance_range_accepted_detections);
            if((all_ROIs(l).center_y()-all_ROIs(l).width()/2.0)/scaleZ < Globals::distance_range_accepted_detections
                    && height > Globals::min_height && height < Globals::max_height)
            {
                Vector<double> bbox(4, 0.0);
                // if there is a valid bounding box
//                if( ROI::GetRegion2D(bbox, camera_origin, points3D_in_ROIs(l)) )
                if(all_ROIs(l).GetRegion2D(bbox, camera_origin, point_cloud) )
                {
                    close_range_BBoxes.pushBack(bbox);

                    Vector<double> distance(2);
                    distance(0) = all_ROIs(l).center_y()/scaleZ;
                    distance(1) = all_ROIs(l).height()/scaleZ;

                    distances.pushBack(distance);
                }
            }
        }
    }



    detected_bounding_boxes = EvaluateTemplate(upper_body_template, depth_map, close_range_BBoxes, distances);
}

void Detector::ComputeFreespace(const Camera& camera,
                     Matrix<int>& occ_map_binary,
                     Matrix<int>& mat_2D_pos_x, Matrix<int>& mat_2D_pos_y,
                     const PointCloud &point_cloud)
{
    // Set Freespace Parameters
    double scale_z_ = Globals::freespace_scaleZ;
    double scale_x_ = Globals::freespace_scaleX;
    double min_x_ = Globals::freespace_minX;
    double min_z_ = Globals::freespace_minZ;
    double max_x_ = Globals::freespace_maxX;
    double max_z_ = Globals::freespace_maxZ;

    int x_bins = (int)round((max_x_ - min_x_)*scale_x_)+1;
    int z_bins = (int)round((max_z_ - min_z_)*scale_z_)+1;

    Matrix<double> occ_map(x_bins, z_bins, 0.0);
    occ_map_binary.set_size(x_bins, z_bins);
    double step_x = (max_x_ - min_x_)/double(x_bins-1);
    double step_z = (max_z_ - min_z_)/double(z_bins-1);

    Vector<double> plane_parameters = camera.get_GP();
    double plane_par_0 = plane_parameters(0);
    double plane_par_1 = plane_parameters(1);
    double plane_par_2 = plane_parameters(2);
    double plane_par_3 = plane_parameters(3)*Globals::WORLD_SCALE;

//    double log_2 = log(2);

    for(int j = 0; j < point_cloud.X.getSize(); j++)
    {
        double zj = point_cloud.Z(j);

        if(zj < Globals::freespace_max_depth_to_cons && zj >= 0.1)
        {
            double xj = point_cloud.X(j);
            double yj = point_cloud.Y(j);

            double distance_to_plane = plane_par_0*xj + plane_par_1*yj + plane_par_2*zj + plane_par_3;

            // Only accept points in upper_body height range (0.1m to 2.5m)
            if(distance_to_plane < 2.5 && distance_to_plane > 0.1)
            {
                double x = xj - min_x_;
                double z = zj - min_z_;

                int pos_x = (int)round(x / step_x);
                int pos_z = (int)round(z / step_z);

//                occ_map(pos_x, pos_z) += z*(log(z) / log_2);
                occ_map(pos_x, pos_z) += z;

                mat_2D_pos_x.data()[j] = pos_x;
                mat_2D_pos_y.data()[j] = pos_z;
            }
        }
    }
    
//    occ_map.WriteToTXT("before.txt");
    
    
     Vector<double> kernel = AncillaryMethods::getGaussian1D(2,3);
     occ_map = AncillaryMethods::conv1D(occ_map, kernel, false);
     occ_map = AncillaryMethods::conv1D(occ_map, kernel, true);

// occ_map.WriteToTXT("after.txt");
 
    int occ_map_length = x_bins*z_bins;
    for(int i = 0; i < occ_map_length; i++)
    {
        occ_map_binary.data()[i] = (occ_map.data()[i] < Globals::freespace_threshold) ? 0 : 1;
    }
}

void Detector::PreprocessROIs(Matrix<int> &labeled_ROIs, Vector<ROI> &all_ROIs)
{
    KConnectedComponentLabeler ccl(Globals::region_size_threshold, labeled_ROIs.data(), labeled_ROIs.x_size(), labeled_ROIs.y_size());

    ccl.Process(); // Main function

    ROI::ExtractROIs(all_ROIs, labeled_ROIs, ccl.m_ObjectNumber);
}

struct ROIBound
{
    ROIBound()
    {
        min_x = min_y = min_z = 1e10;
        max_x = max_y = max_z = -1e10;
    }

//    int ind_min_x, ind_max_x;
//    int ind_min_y, ind_max_y;
//    int ind_min_z, ind_max_z;
    double min_x, max_x;
    double min_y, max_y;
    double min_z, max_z;
};

void Detector::ExtractPointsInROIs(/*Vector<Vector<Vector<double> > > &all_points_in_ROIs*/Vector<ROI> &all_ROIs, const Matrix<int> &mat_2D_pos_x, const Matrix<int> &mat_2D_pos_y, const PointCloud& point_cloud, const Matrix<int> &labeled_ROIs)
{
//    int number_of_ROIs = all_points_in_ROIs.getSize();
    int number_of_ROIs = all_ROIs.getSize();

//    for(int i = 0; i < number_of_ROIs; i++)
//    {
//        all_points_in_ROIs(i).reserveCapacity(50000);
//    }

    Vector<ROIBound> min_maxs(number_of_ROIs);
    ROIBound* roi_bound;
    int roi_ind;
    int mat_size = mat_2D_pos_x.x_size()*mat_2D_pos_x.y_size();
    for(int i = 0; i < mat_size; i++)
    {
        if(mat_2D_pos_x.data()[i] >= 0)
        {
            roi_ind = labeled_ROIs(mat_2D_pos_x.data()[i], mat_2D_pos_y.data()[i])-1;
            if(roi_ind > -1)
            {
//                all_points_in_ROIs(pos_in_proj).pushBack(Vector<double>(point_cloud.X(i), point_cloud.Y(i), point_cloud.Z(i)));
                all_ROIs(roi_ind).has_any_point = true;
                double x = point_cloud.X(i), y = point_cloud.Y(i), z = point_cloud.Z(i);
                roi_bound = &min_maxs(roi_ind);
                if(roi_bound->min_x > x)
                {
                    roi_bound->min_x = x;
                    all_ROIs(roi_ind).ind_min_x = i;
                }
                if(roi_bound->max_x < x)
                {
                    roi_bound->max_x = x;
                    all_ROIs(roi_ind).ind_max_x = i;
                }
                if(roi_bound->min_y > y)
                {
                    roi_bound->min_y = y;
                    all_ROIs(roi_ind).ind_min_y = i;
                }
                if(roi_bound->max_y < y)
                {
                    roi_bound->max_y = y;
                    all_ROIs(roi_ind).ind_max_y = i;
                }
                if(roi_bound->min_z > z)
                {
                    roi_bound->min_z = z;
                    all_ROIs(roi_ind).ind_min_z = i;
                }
                if(roi_bound->max_z < z)
                {
                    roi_bound->max_z = z;
                    all_ROIs(roi_ind).ind_max_z = i;
                }
            }
        }
    }
}

Vector<Vector<double> >  Detector::EvaluateTemplate(const Matrix<double> &upper_body_template,
                                                        const Matrix<double> &depth_map,
                                                        Vector<Vector<double> > &close_range_BBoxes,
                                                        Vector<Vector<double> > distances)
{
    int stride = Globals::evaluation_stride;
    int nr_scales = Globals::evaluation_nr_scales;
    int inc_cropped_height = Globals::evaluation_inc_cropped_height;

    // performance helper variables: just for avoiding recalculation
    int int_half_template_size = Globals::template_size / 2;
    double double_half_template_size = Globals::template_size / 2.0;

    Vector<Vector<double> > final_result;

    // generate the scales
    Vector<double> all_scales(nr_scales, 1.0);
    all_scales(0) = 1;
    for(int sc = 1; sc < nr_scales; ++sc)
    {
        all_scales(sc) = pow(Globals::evaluation_scale_stride,sc);
    }

//    Matrix<double> roi_img(Globals::dImWidth, Globals::dImHeight, 0);
    if(visualize_roi)
        roi_image = Matrix<int>(Globals::dImWidth, Globals::dImHeight, 0);

    for (int i = 0; i < close_range_BBoxes.getSize(); i++)
    {
        Vector<Vector<double> > result;

        int cropped_height = (int)(close_range_BBoxes(i)(3)/2.0);
        cropped_height += (close_range_BBoxes(i)(3) * Globals::evaluation_inc_height_ratio)/2.0;
        close_range_BBoxes(i)(1) -= (close_range_BBoxes(i)(3) * Globals::evaluation_inc_height_ratio)/2.0;

        if( close_range_BBoxes(i)(1)+cropped_height >= Globals::dImHeight)
            cropped_height = Globals::dImHeight - (int)close_range_BBoxes(i)(1) - 1;

        ROS_DEBUG("(distances(i) %f radius %f", distances(i)(0), distances(i)(1)/2.0);
//        if(Globals::verbose)
//            cout << "(distances(i) " << distances(i)(0) << " radius " << distances(i)(1)/2.0 << endl;

        // Cropped and Filter depth_map with respect to distance from camera
        int start_column = (int)close_range_BBoxes(i)(0);
        int end_column = (int)(close_range_BBoxes(i)(0) + close_range_BBoxes(i)(2));
        int start_row = (int)max(0.0, close_range_BBoxes(i)(1));
        int end_row = (int)close_range_BBoxes(i)(1) + cropped_height;

        Matrix<double> cropped(end_column-start_column+1, end_row-start_row+1);

        double min_distance_threshold = distances(i)(0)- (distances(i)(1)+0.2)/2.0;
        double max_distance_threshold = distances(i)(0)+ (distances(i)(1)+0.2)/2.0;
        for(int ii = 0, ii_depth = start_column; ii < cropped.x_size(); ii++, ii_depth++)
        {
            for(int jj = 0, jj_depth = start_row; jj < cropped.y_size(); jj++, jj_depth++)
            {
                if(depth_map(ii_depth,jj_depth) <= min_distance_threshold || depth_map(ii_depth,jj_depth) >= max_distance_threshold)
                {
                    cropped(ii, jj) = 0;
                }
                else
                {
                    cropped(ii, jj) = depth_map(ii_depth, jj_depth);
                }
            }
        }

//        roi_img.insert(cropped,start_column,start_row);
        ////////////////
        if(visualize_roi)
        {
            for(int tmpx=start_column, tmpxx=0; tmpxx<cropped.x_size(); tmpx++,tmpxx++)
            {
                for(int tmpy=start_row, tmpyy=0; tmpyy<cropped.y_size(); tmpy++,tmpyy++)
                {
                    if(tmpyy==0 || tmpyy==cropped.y_size()-1 || tmpxx==0 || tmpxx==cropped.x_size()-1)
                        roi_image(tmpx,tmpy)=i+1;

                    if(cropped(tmpxx,tmpyy)!=0)
                        roi_image(tmpx,tmpy)=i+1;
                }
            }
        }
        /////////////////////////////////

        // Resize Cropped - with respect to template
        double ratio = close_range_BBoxes(i)(3) / (Globals::template_size * 3.0);
        int new_height = (int)(cropped.y_size() * all_scales(nr_scales-1) / ratio);
        int new_width = (int)(cropped.x_size() * all_scales(nr_scales-1) / ratio);
        if(new_height<=0 || new_width<=0)
            continue;
        if(cropped.y_size() > new_height)
        {
            cropped.DownSample(new_width, new_height);
        }
        else if(cropped.y_size() < new_height)
        {
            cropped.UpSample(new_width, new_height);
        }


        //*******************************************************************************************************
        Matrix<int> b_cropped(cropped.x_size(), cropped.y_size());
        for(int ic = 0; ic<cropped.x_size(); ++ic)
        {
            for(int j=0; j<cropped.y_size(); ++j)
            {
                if(cropped(ic,j)>0)
                    b_cropped(ic,j)=1;
                else
                    b_cropped(ic,j)=0;
            }
        }

        KConnectedComponentLabeler ccl(Globals::region_size_threshold, b_cropped.data(), b_cropped.x_size(), b_cropped.y_size());

        ccl.Process();

        Matrix<double> components(cropped.x_size(),cropped.y_size());
        //*******************************************************************************************************
        for(int cr_com = 0; cr_com < ccl.m_ObjectNumber; ++cr_com)
        {
            for(int x=0; x<cropped.x_size(); ++x)
            {
                for(int y=0; y<cropped.y_size(); ++y)
                {
                    if(b_cropped(x,y)==cr_com+1)
                        components(x,y) = cropped(x,y);
                    else
                        components(x,y) = 0;
                }
            }

            Matrix<double> copy_component = components;

            for(int scale_index = 0; scale_index < all_scales.getSize(); scale_index++)
            {
                copy_component = components;

                // Resize Cropped in loop with different scales
                int xSizeCropped = (int)(copy_component.x_size() / all_scales(scale_index));
                int ySizeCropped = (int)(copy_component.y_size() / all_scales(scale_index));

                if(all_scales(scale_index) != 1)
                    copy_component.DownSample(xSizeCropped , ySizeCropped);

                Matrix<double> extended_cropped(xSizeCropped + Globals::template_size, ySizeCropped+inc_cropped_height, 0.0);
                extended_cropped.insert(copy_component, (int)(double_half_template_size)-1, 0);

                //*Local Max *******************************
                Matrix<double> extended_cropped_open = extended_cropped;
                AncillaryMethods::MorphologyOpen(extended_cropped_open);
                Vector<double> ys ,slopes;
                AncillaryMethods::ExtractSlopsOnBorder(extended_cropped_open, ys, slopes);
                Vector<int> max_xs = AncillaryMethods::FindLocalMax(slopes);
                //******************************************

                int distances_matrix_x_size = max_xs.getSize();

                Vector<double> resulted_distances(distances_matrix_x_size);
                Vector<double> resulted_medians(distances_matrix_x_size);

                Vector<int> rxs(max_xs.getSize()),rys(max_xs.getSize());
                for(int ii = 0; ii<max_xs.getSize(); ++ii)
                {
                    int cx = max(max_xs(ii)-int_half_template_size,0);
                    int cy = max(extended_cropped.y_size()-(int)ys(max_xs(ii))-4, 0);
                    rxs(ii) = cx;
                    rys(ii) = cy;
                    double local_result, local_best=1000;

                    for(int y=cy; y<=cy+1*stride; y+=stride)
                    {
                        if(y>=extended_cropped.y_size() || y>=extended_cropped.y_size()) continue;
                        int y_size = min(extended_cropped.y_size()-1, y+Globals::template_size) - y;

                        int start_row = (int)max(0.0, y + y_size/2.0-5);
                        int end_row = (int)min((double)Globals::dImHeight-1, y + y_size/2.0+5);
                        if(end_row >=extended_cropped.y_size()) continue;

                        for(int x=max(0,cx-5*stride); x<=cx+5*stride; x+=stride)
                        {
                            if(x>=extended_cropped.x_size()) continue;
                            int x_size = min(extended_cropped.x_size()-1, x+Globals::template_size) - x;

                            int start_column = (int)max(0.0, x + x_size/2.0-5);
                            int end_column = (int)min((double)Globals::dImWidth-1, x + x_size/2.0+5);

                            // Normalize the cropped part of the image. regarding the current position of the template;
                            // Crop only some pixels in the middle
                            double median = AncillaryMethods::MedianOfMatrixRejectZero(extended_cropped, start_row, end_row, start_column, end_column);
                            if(median == 0)
                            {
                                resulted_distances(ii) = 1000;
                                continue;
                            }
//                            extended_cropped *= 1.0/median;

                            int x_start_of_temp = max(0, int_half_template_size - x);
                            double x_end_of_temp = Globals::template_size;
                            int evaluating_area = (x_end_of_temp - x_start_of_temp)*Globals::template_size+1;

                            double sum = 0;

                            if(evaluating_area > Globals::template_size * double_half_template_size)
                            {
                                for(int x_of_temp = 0; x_of_temp < x_end_of_temp; x_of_temp++)
                                {
                                    int x_of_extended_cropp = x + x_of_temp;

                                    for(int y_of_temp = 0; y_of_temp < Globals::template_size; y_of_temp++)
                                    {
                                        double difference = upper_body_template(x_of_temp, y_of_temp)-extended_cropped(x_of_extended_cropp, y_of_temp+y)/median;

                                        sum += difference*difference;
                                    }
                                }

                                local_result = sum/(double)evaluating_area;
                                if(local_best>local_result)
                                {
                                    local_best = local_result;
                                    resulted_medians(ii) = median;
                                    rxs(ii)=x;
                                    rys(ii)=y;
                                }
                            }
                        }
                    }
                    resulted_distances(ii) = local_best;
                }

                int n_xSizeTemp = (int)(Globals::template_size*ratio/all_scales(scale_index));

//                double n_threshold = 0.15;
                for(int ii =0; ii<resulted_distances.getSize(); ++ii)
                {
                    if(resulted_distances(ii)<Globals::evaluation_NMS_threshold_LM)
                    {
                        int x = rxs(ii);
                        int y = rys(ii);

                        Vector<double> bbox(6);
                        bbox(0) = (x-double_half_template_size)*ratio/all_scales(scale_index) + close_range_BBoxes(i)(0);
                        bbox(1) = y*ratio/all_scales(scale_index) +close_range_BBoxes(i)(1);
                        bbox(2) = n_xSizeTemp;
                        bbox(3) = n_xSizeTemp;
                        bbox(4) = resulted_distances(ii);
                        bbox(5) = resulted_medians(ii);

                        result.pushBack(bbox);
                    }
                }
            }
        }
        AncillaryMethods::GreedyNonMaxSuppression(result, Globals::evaluation_greedy_NMS_overlap_threshold, Globals::evaluation_greedy_NMS_threshold, upper_body_template, final_result);
    }
//    roi_img.WriteToTXT("roi_img.txt");
    return final_result;
}
