#ifndef DETECTED_PERSONS_DISPLAY_H
#define DETECTED_PERSONS_DISPLAY_H

#include <map>
#include <boost/circular_buffer.hpp>

#include <spencer_tracking_msgs/DetectedPersons.h>

#include "person_display_common.h"

namespace spencer_tracking_rviz_plugin
{
    /// The visual of a tracked person.
    struct DetectedPersonVisual
    {
        shared_ptr<Ogre::SceneNode> sceneNode;

        shared_ptr<PersonVisual> personVisual;
        shared_ptr<TextNode> detectionIdText, confidenceText, modalityText;
        shared_ptr<rviz::Arrow> orientationArrow;
        shared_ptr<CovarianceVisual> covarianceVisual;

        float confidence;
        bool hasValidOrientation;
        unsigned int detectionId;
    };

    // The DetectedPersonsDisplay class itself just implements a circular buffer,
    // editable parameters, and Display subclass machinery.
    class DetectedPersonsDisplay: public PersonDisplayCommon<spencer_tracking_msgs::DetectedPersons>
    {
    Q_OBJECT
    public:
        // Constructor.  pluginlib::ClassLoader creates instances by calling
        // the default constructor, so make sure you have one.
        DetectedPersonsDisplay() {};
        virtual ~DetectedPersonsDisplay();

        // Overrides of protected virtual functions from Display.  As much
        // as possible, when Displays are not enabled, they should not be
        // subscribed to incoming data and should not show anything in the
        // 3D view.  These functions are where these connections are made
        // and broken.
        
        virtual void onInitialize();

    protected:
        // A helper to clear this display back to the initial state.
        virtual void reset();

        // Must be implemented by derived classes because MOC doesn't work in templates
        virtual rviz::DisplayContext* getContext() {
            return context_;
        }

    private Q_SLOTS:
        void personVisualTypeChanged();

        // Called whenever one of the properties in PersonDisplayCommonProperties has been changed
        virtual void stylesChanged();

    private:
        // Function to handle an incoming ROS message.
        void processMessage(const spencer_tracking_msgs::DetectedPersons::ConstPtr& msg);
       
        // All currently active tracks, with unique track ID as map key
        vector<shared_ptr<DetectedPersonVisual> > m_previousDetections;

        // Properties
        rviz::BoolProperty* m_render_covariances_property;
        rviz::BoolProperty* m_render_detection_ids_property;
        rviz::BoolProperty* m_render_confidences_property;
        rviz::FloatProperty* m_low_confidence_threshold_property;
        rviz::FloatProperty* m_low_confidence_alpha_property;
        rviz::BoolProperty* m_render_orientations_property;
        rviz::BoolProperty* m_render_modality_text_property;

        rviz::FloatProperty* m_text_spacing_property;
        rviz::FloatProperty* m_covariance_line_width_property;
    };

} // end namespace spencer_tracking_rviz_plugin

#endif // DETECTED_PERSONS_DISPLAY_H
