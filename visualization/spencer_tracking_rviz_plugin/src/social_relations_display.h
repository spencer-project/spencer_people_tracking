#ifndef SOCIAL_RELATIONS_DISPLAY_H
#define SOCIAL_RELATIONS_DISPLAY_H

#include <spencer_social_relation_msgs/SocialRelations.h>

#include "person_display_common.h"
#include "tracked_persons_cache.h"

namespace spencer_tracking_rviz_plugin
{
    /// The display which can be added in RViz to display social relations.
    class SocialRelationsDisplay: public PersonDisplayCommon<spencer_social_relation_msgs::SocialRelations>
    {
    Q_OBJECT
    public:
        // Constructor.  pluginlib::ClassLoader creates instances by calling
        // the default constructor, so make sure you have one.
        SocialRelationsDisplay() {};
        virtual ~SocialRelationsDisplay();

        // Overrides of protected virtual functions from Display.  As much
        // as possible, when Displays are not enabled, they should not be
        // subscribed to incoming data and should not show anything in the
        // 3D view.  These functions are where these connections are made
        // and broken.

        // Called after the constructors have run
        virtual void onInitialize();

    protected:
        // A helper to clear this display back to the initial state.
        virtual void reset();

        // Must be implemented by derived classes because MOC doesn't work in templates
        virtual rviz::DisplayContext* getContext() {
            return context_;
        }

    private:
        struct RelationVisual {
            std::string type;
            double relationStrength;
            shared_ptr<rviz::BillboardLine> relationLine;
            shared_ptr<TextNode> relationText;
            track_id trackId1, trackId2; // required to hide certain tracks
        };

        // Functions to handle an incoming ROS message.
        void processMessage(const spencer_social_relation_msgs::SocialRelations::ConstPtr& msg);
        
        // Helper functions
        void updateRelationVisualStyles(shared_ptr<RelationVisual>& relationVisual);
        
        // Scene node for group affiliation history visualization
        shared_ptr<Ogre::SceneNode> m_socialRelationsSceneNode;

        // User-editable property variables.
        rviz::StringProperty* m_relation_type_filter_property;
        
        rviz::BoolProperty* m_render_positive_person_relations_property;
        rviz::BoolProperty* m_render_negative_person_relations_property;

        rviz::FloatProperty* m_positive_person_relation_threshold;
        rviz::ColorProperty* m_positive_person_relations_color;
        rviz::ColorProperty* m_negative_person_relations_color;

        // State variables
        vector<shared_ptr<RelationVisual> > m_relationVisuals;
        TrackedPersonsCache m_trackedPersonsCache;

    private Q_SLOTS:
        virtual void stylesChanged();
    };

} // end namespace spencer_tracking_rviz_plugin

#endif // SOCIAL_RELATIONS_DISPLAY_H
