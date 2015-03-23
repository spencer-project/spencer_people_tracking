#ifndef PERSON_VISUAL_H
#define PERSON_VISUAL_H

#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include <OgreSceneNode.h>
#include <OgreAnimation.h>
#include <OgreSharedPtr.h>
#include <OgreEntity.h>


namespace spencer_tracking_rviz_plugin {
    // Abstract class for visuals which have got an adjustable line width
    class HasLineWidth {
    public:
        virtual void setLineWidth(double lineWidth) = 0;
    };

    // Default arguments that need to be supplied to all types of PersonVisual
    struct PersonVisualDefaultArgs {
        PersonVisualDefaultArgs(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode) : sceneManager(sceneManager), parentNode(parentNode) {}
        Ogre::SceneManager* sceneManager;
        Ogre::SceneNode* parentNode;
    };

    /// Base class for all person visualization types
    class PersonVisual {
    public:
        PersonVisual(const PersonVisualDefaultArgs& args) :
                m_sceneManager(args.sceneManager),
                m_correctOrientation( Ogre::Degree(90), Ogre::Vector3(1,0,0) )
        {
            m_parentSceneNode = args.parentNode;
            m_sceneNode = args.parentNode->createChildSceneNode();

            Ogre::Vector3 scale(1,1,1);
            m_sceneNode->setScale(m_correctOrientation * scale);
        }

        virtual ~PersonVisual() {
            m_sceneManager->destroySceneNode(m_sceneNode->getName());
        };

        void setPosition(const Ogre::Vector3& position) {
            m_sceneNode->setPosition(position);
        }

        const Ogre::Vector3& getPosition() const {
            return m_sceneNode->getPosition();
        }

        void setOrientation(const Ogre::Quaternion& orientation) {
            m_sceneNode->setOrientation(orientation * m_correctOrientation);
        }

        const Ogre::Quaternion& getOrientation() const {
           return m_sceneNode->getOrientation();
        }

        virtual void setScalingFactor(double scalingFactor) {
            m_sceneNode->setScale(scalingFactor, scalingFactor, scalingFactor);
        }

        void setVisible(bool visible) {
            m_sceneNode->setVisible(visible, true);
        }

        Ogre::SceneNode* getParentSceneNode() {
            return m_parentSceneNode;
        }

        virtual void update(float deltaTime) {}
        virtual void setColor(const Ogre::ColourValue& c) = 0;
        virtual double getHeight() = 0;

    protected:
        Ogre::SceneManager* m_sceneManager;
        Ogre::SceneNode *m_sceneNode, *m_parentSceneNode;
        Ogre::Quaternion m_correctOrientation;
    };


    /// Visualization of a person as cylinder (body) + sphere (head)
    class CylinderPersonVisual : public PersonVisual {
    public:
        CylinderPersonVisual(const PersonVisualDefaultArgs& args) : PersonVisual(args)
        {
            m_bodyShape = new rviz::Shape(rviz::Shape::Cylinder, args.sceneManager, m_sceneNode);
            m_headShape = new rviz::Shape(rviz::Shape::Sphere, args.sceneManager, m_sceneNode);

            const float headDiameter = 0.4;
            const float totalHeight = getHeight();
            const float cylinderHeight = totalHeight - headDiameter;

            m_bodyShape->setScale(Ogre::Vector3(headDiameter, headDiameter, cylinderHeight));
            m_headShape->setScale(Ogre::Vector3(headDiameter, headDiameter, headDiameter));

            m_bodyShape->setPosition(Ogre::Vector3(0, 0, cylinderHeight / 2 - totalHeight / 2));
            m_headShape->setPosition(Ogre::Vector3(0, 0, totalHeight / 2 - headDiameter / 2 ));
        }

        virtual ~CylinderPersonVisual() {
            delete m_bodyShape;
            delete m_headShape;
        }

        virtual void setColor(const Ogre::ColourValue& c) {
            m_bodyShape->setColor(c);
            m_headShape->setColor(c);
        }

        virtual double getHeight() {
            return 1.75;
        }

    private:
        rviz::Shape *m_bodyShape, *m_headShape;
    };


    /// Visualization of a person as a wireframe bounding box
    class BoundingBoxPersonVisual : public PersonVisual, public HasLineWidth {
    public:
        BoundingBoxPersonVisual(const PersonVisualDefaultArgs& args, double height = 1.75, double width = 0.6, double scalingFactor = 1.0) : PersonVisual(args)
        {
            m_width = width; m_height = height; m_scalingFactor = scalingFactor; m_lineWidth = 0.03;
            m_wireframe = NULL;
            generateWireframe();
        }

        virtual ~BoundingBoxPersonVisual() {
            delete m_wireframe;
        }

        virtual void setColor(const Ogre::ColourValue& c) {
            m_wireframe->setColor(c.r, c.g, c.b, c.a);
        }

        virtual double getHeight() {
            return m_height;
        }

        virtual void setLineWidth(double lineWidth) {
            m_wireframe->setLineWidth(lineWidth);
        }

        /*
        virtual void setScalingFactor(double scalingFactor) {
            if(scalingFactor != m_scalingFactor) {
                m_scalingFactor = scalingFactor;
                generateWireframe();
            }
        }
        */

    protected:
        virtual void generateWireframe() {
            delete m_wireframe;
            m_wireframe = new rviz::BillboardLine(m_sceneManager, m_sceneNode);
            
            m_wireframe->setLineWidth(m_lineWidth);   
            m_wireframe->setMaxPointsPerLine(2);
            m_wireframe->setNumLines(12);

            double w = m_width * m_scalingFactor, h = m_height * m_scalingFactor;
            Ogre::Vector3 bottomLeft(0, -w, 0), bottomRight(0, 0, 0), topLeft(0, -w, h), topRight(0, 0, h);
            Ogre::Vector3 rear(w, 0, 0);

            // Front quad
                                        m_wireframe->addPoint(bottomLeft);          m_wireframe->addPoint(bottomRight);
            m_wireframe->newLine();     m_wireframe->addPoint(bottomRight);         m_wireframe->addPoint(topRight);
            m_wireframe->newLine();     m_wireframe->addPoint(topRight);            m_wireframe->addPoint(topLeft);
            m_wireframe->newLine();     m_wireframe->addPoint(topLeft);             m_wireframe->addPoint(bottomLeft);

            // Rear quad
            m_wireframe->newLine();     m_wireframe->addPoint(bottomLeft + rear);   m_wireframe->addPoint(bottomRight + rear);
            m_wireframe->newLine();     m_wireframe->addPoint(bottomRight + rear);  m_wireframe->addPoint(topRight + rear);
            m_wireframe->newLine();     m_wireframe->addPoint(topRight + rear);     m_wireframe->addPoint(topLeft + rear);
            m_wireframe->newLine();     m_wireframe->addPoint(topLeft + rear);      m_wireframe->addPoint(bottomLeft + rear);

            // Four connecting lines between front and rear
            m_wireframe->newLine();     m_wireframe->addPoint(bottomLeft);          m_wireframe->addPoint(bottomLeft + rear);
            m_wireframe->newLine();     m_wireframe->addPoint(bottomRight);         m_wireframe->addPoint(bottomRight + rear);
            m_wireframe->newLine();     m_wireframe->addPoint(topRight);            m_wireframe->addPoint(topRight + rear);
            m_wireframe->newLine();     m_wireframe->addPoint(topLeft);             m_wireframe->addPoint(topLeft + rear);
        
            m_wireframe->setPosition(Ogre::Vector3(-w/2, w/2, -h/2));
        } 

    private:
        rviz::BillboardLine *m_wireframe;
        double m_width, m_height, m_scalingFactor, m_lineWidth;
    };


    /// Visualization of a person as a mesh (walking human)
    class MeshPersonVisual : public PersonVisual {
    public:
        MeshPersonVisual(const PersonVisualDefaultArgs& args);

        virtual ~MeshPersonVisual();

        virtual void update(float deltaTime);

        virtual void setColor(const Ogre::ColourValue& c);

        void setAnimationState(const std::string& nameOfAnimationState);

        void setWalkingSpeed(float walkingSpeed);

        virtual double getHeight() {
            return 1.75;
        }

        virtual void setScalingFactor(double scalingFactor) {
            // Not supported (for some reason causes the mesh to be mirrored vertically).
        }

    private:
        Ogre::SceneNode *m_childSceneNode;
        Ogre::Entity* entity_;
        Ogre::AnimationState* m_animationState;
        std::set<Ogre::MaterialPtr> materials_;
        float m_walkingSpeed;
    };

}

#endif // PERSON_VISUAL_H
