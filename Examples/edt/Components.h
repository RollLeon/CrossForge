#ifndef CFORGESANDBOX_COMPONENTS_H
#define CFORGESANDBOX_COMPONENTS_H

#include <Eigen/Core>
#include "behaviortree_cpp/bt_factory.h"
#include "crossforge/Graphics/Actors/IRenderableActor.h"
#include <queue>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <flecs.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include "crossforge/Graphics/SceneGraph/SGNTransformation.h"


namespace CForge {
    class AIComponent {
    public:
        BT::Tree tree;
    };

    class GeometryComponent {
    public:
        IRenderableActor *actor;

        void init(IRenderableActor *pActor) {
            actor = pActor;
        }
    };

    class ObstacleComponent {

    };

    enum GameState {
        GAMEPLAY,
        DIALOG            // DIALOG: Maus nicht disabled, sodass mit imgui interagiert werden kann, Kamera starr, Spieler reagiert nicht auf Tasteneingaben wie WASD
    };                              // GAMEPLAY: Cursor gefangen, Kamera beweglich, Spieler kann sich bewegen
    class PlayerComponent {
    public:
        constexpr static const float HEIGHT = 2;
        GameState gameState;
        VirtualCamera *pCamera;
        Keyboard *pKeyboard;
        Mouse *pMouse;

        explicit PlayerComponent(VirtualCamera *camera, Keyboard *keyboard, Mouse *mouse) {
            gameState = GAMEPLAY;
            pCamera = camera;
            pKeyboard = keyboard;
            pMouse = mouse;
        }
    };

    class PathComponent {
    public:
        std::queue<Eigen::Vector3f> path;

        PathComponent() {
            path = std::queue<Eigen::Vector3f>();
        };
    };


    class PathRequestComponent {

    public:
        Eigen::Vector3f start;
        Eigen::Vector3f destination;
    };

    class PhysicsComponent {
    public:
        std::unique_ptr<btRigidBody> collisionObject;

        explicit PhysicsComponent(btRigidBody* co) {
            collisionObject = std::unique_ptr<btRigidBody>(co);
        };
    };

    class PlantComponent {
    public:
        float waterLevel;
        float maxWaterLevel = 10.0;
    };

    class PositionComponent {
    public:
        Eigen::Vector3f m_Translation;
        Eigen::Quaternionf m_Rotation;
        Eigen::Vector3f m_Scale;
        Eigen::Vector3f m_TranslationDelta;
        Eigen::Quaternionf m_RotationDelta;
        Eigen::Vector3f m_ScaleDelta;

        void init() {
            m_Translation = Eigen::Vector3f::Zero();
            m_Rotation = Eigen::Quaternionf::Identity();
            m_Scale = Eigen::Vector3f::Ones();
            m_TranslationDelta = Eigen::Vector3f::Zero();
            m_RotationDelta = Eigen::Quaternionf::Identity();
            m_ScaleDelta = Eigen::Vector3f::Zero();
        }

        void translation(Eigen::Vector3f Translation) {
            m_Translation = Translation;
        }//translation

        void rotation(Eigen::Quaternionf Rotation) {
            m_Rotation = Rotation;
        }//rotation

        void scale(Eigen::Vector3f Scale) {
            m_Scale = Scale;
        }//scale

        void translationDelta(Eigen::Vector3f TranslationDelta) {
            m_TranslationDelta = TranslationDelta;
        }//translationDelta

        void rotationDelta(Eigen::Quaternionf RotationDelta) {
            m_RotationDelta = RotationDelta;
        }//rotationDelta

        void scaleDelta(Eigen::Vector3f ScaleDelta) {
            m_ScaleDelta = ScaleDelta;
        }//scaleDelta

        Eigen::Vector3f translation(void) const {
            return m_Translation;
        }//translation

        Eigen::Quaternionf rotation(void) const {
            return m_Rotation;
        }//rotation

        Eigen::Vector3f scale(void) const {
            return m_Scale;
        }//scale

        Eigen::Vector3f translationDelta(void) const {
            return m_TranslationDelta;
        }//translationDelta

        Eigen::Quaternionf rotationDelta(void) const {
            return m_RotationDelta;
        }//rotationDelta

        Eigen::Vector3f scaleDelta(void) const {
            return m_ScaleDelta;
        }//scaleDelta

        void update(float FPSScale) {

            m_Translation += FPSScale * m_TranslationDelta;
            float Temp = FPSScale;
            Eigen::Quaternionf TargetRot = m_Rotation;
            while (Temp > 1.0f) {
                TargetRot *= m_RotationDelta;
                Temp -= 1.0f;
            }
            m_Rotation = TargetRot.slerp(Temp, TargetRot * m_RotationDelta);
            m_Scale += FPSScale * m_ScaleDelta;

        }//update

        void buildTansformation(Eigen::Vector3f *pPosition, Eigen::Quaternionf *pRotation, Eigen::Vector3f *pScale) {
            Eigen::Vector3f ParentPosition = Eigen::Vector3f::Zero();
            Eigen::Quaternionf ParentRotation = Eigen::Quaternionf::Identity();
            Eigen::Vector3f ParentScale = Eigen::Vector3f::Ones();

            if (nullptr != pPosition) (*pPosition) = ParentPosition + ParentRotation * m_Translation;
            if (nullptr != pRotation) (*pRotation) = ParentRotation * m_Rotation;
            if (nullptr != pScale) (*pScale) = ParentScale.cwiseProduct(m_Scale);

        }//buildTrnasformation
    };

    class SteeringComponent {
    public:
        float max_force;
        float max_speed;
        float mass;
        float securityDistance;
        enum drivingMode {
            PathFollowing,
            Seeking,
            TurnTo
        };
        Eigen::Vector3f originalTarget; // workaround for blackboard in behaviour tree
        Eigen::Vector3f targetRotation;
        drivingMode mode;
    };

}

#endif //CFORGESANDBOX_COMPONENTS_H
