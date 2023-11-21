//
// Created by private on 18.11.23.
//

#ifndef CFORGESANDBOX_PHYSICSCOMPONENT_H
#define CFORGESANDBOX_PHYSICSCOMPONENT_H

#include <BulletCollision/CollisionDispatch/btCollisionObject.h>

namespace CForge {
    class PhysicsComponent {
    public:
        std::unique_ptr<btRigidBody> collisionObject;

        explicit PhysicsComponent(btRigidBody *co) {
            collisionObject = std::unique_ptr<btRigidBody>(co);
        };
    };
}
#endif //CFORGESANDBOX_PHYSICSCOMPONENT_H
