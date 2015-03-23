#ifndef TEXT_NODE_H
#define TEXT_NODE_H

#include <rviz/ogre_helpers/shape.h>

namespace spencer_tracking_rviz_plugin {
    class TextNode {
    public:
        TextNode(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode, Ogre::Vector3 position = Ogre::Vector3::ZERO) : m_sceneManager(sceneManager)
        {
            m_sceneNode = parentNode->createChildSceneNode();

            m_text = new rviz::MovableText("text");
            m_text->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_BELOW);
            m_sceneNode->attachObject(m_text);

            setCharacterHeight(1.0);
            setPosition(position);
            setVisible(true);
        }

        virtual ~TextNode() {
            m_sceneManager->destroySceneNode(m_sceneNode->getName());
            delete m_text;
        };

        void setCharacterHeight(double characterHeight) {
            m_text->setCharacterHeight(characterHeight);
            m_text->setSpaceWidth(0.3 * characterHeight);
        }

        double getCharacterHeight() {
            return m_text->getCharacterHeight();
        }

        void setCaption(const std::string& caption) {
            m_text->setCaption(caption);
        }

        void setPosition(const Ogre::Vector3& position) {
            m_sceneNode->setPosition(position);
        }

        void setVisible(bool visible) {
            m_sceneNode->setVisible(visible, true);
        }

        void setColor(const Ogre::ColourValue& c) {
            m_text->setColor(c);
        }

        void showOnTop(bool onTop = true) {
            m_text->showOnTop(onTop);
        }

    private:
        Ogre::SceneManager* m_sceneManager;
        Ogre::SceneNode* m_sceneNode;
        rviz::MovableText* m_text;
    };

}

#endif // TEXT_NODE_H
