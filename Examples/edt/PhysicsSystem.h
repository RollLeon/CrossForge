//
// Created by private on 18.11.23.
//

#ifndef CFORGESANDBOX_PHYSICSSYSTEM_H
#define CFORGESANDBOX_PHYSICSSYSTEM_H

#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/BroadphaseCollision/btAxisSweep3.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <flecs.h>
#include <eigen3/Eigen/Geometry>
#include "PhysicsComponent.h"
#include "PositionComponent.h"

namespace CForge {
    class PhysicsSystem {
    public:
        static void addPhysicsSystem(flecs::world &world) {
            btCollisionConfiguration *bt_collision_configuration;
            btCollisionDispatcher *bt_dispatcher;
            btBroadphaseInterface *bt_broadphase;
            btDiscreteDynamicsWorld *dynamicsWorld;

            double scene_size = 500;
            unsigned int max_objects = 16000;

            bt_collision_configuration = new btDefaultCollisionConfiguration();
            bt_dispatcher = new btCollisionDispatcher(bt_collision_configuration);

            btScalar sscene_size = (btScalar) scene_size;
            btVector3 worldAabbMin(-sscene_size, -sscene_size, -sscene_size);
            btVector3 worldAabbMax(sscene_size, sscene_size, sscene_size);
//This is one type of broadphase, bullet has others that might be faster depending on the application
            bt_broadphase = new bt32BitAxisSweep3(worldAabbMin, worldAabbMax, max_objects, 0,
                                                  true);  // true for disabling raycast accelerator
            ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
            btSequentialImpulseConstraintSolver *solver = new btSequentialImpulseConstraintSolver;

            dynamicsWorld = new btDiscreteDynamicsWorld(bt_dispatcher, bt_broadphase, solver,
                                                        bt_collision_configuration);

            dynamicsWorld->setGravity(btVector3(0, -10, 0));
            world.system<PhysicsComponent, PositionComponent>("PhysicsSystem")
                    .iter([&dynamicsWorld](flecs::iter it, PhysicsComponent *ps, PositionComponent *pc) {
                        for (size_t i = 0; i < it.count(); i++) {
                            auto pos = pc[i].translation();
                            auto vel = pc[i].translationDelta();
                            Eigen::Quaternionf rot = pc[i].rotation();
                            ps[i].collisionObject->getWorldTransform().setOrigin(btVector3(pos.x(), pos.y(), pos.z()));
                            ps[i].collisionObject->setInterpolationLinearVelocity(btVector3(vel.x(), vel.y(), vel.z()));
                            ps[i].collisionObject->getWorldTransform().setRotation(
                                    btQuaternion(rot.x(), rot.y(), rot.z(), rot.w()));
                        }
                        dynamicsWorld->stepSimulation(it.delta_time());
                        for (size_t i = 0; i < it.count(); i++) {

                            auto rot = ps[i].collisionObject->getWorldTransform().getRotation();
                            auto pos = ps[i].collisionObject->getWorldTransform().getOrigin();
                            auto vel = ps[i].collisionObject->getInterpolationLinearVelocity();

                            pc[i].rotation(Eigen::Quaternionf(rot.w(), rot.x(), rot.y(), rot.z()));
                            pc[i].translation(Eigen::Vector3f(pos.x(), pos.y(), pos.z()));
                            pc[i].translationDelta(Eigen::Vector3f(vel.x(), vel.y(), vel.z()));
                        }
                    });
            world.observer<PhysicsComponent>("Physics Body Add Observer")
                    .event(flecs::OnAdd)
                    .each([dynamicsWorld](PhysicsComponent &pc) {
                        std::cout << "Added physics component" << std::endl;
                        dynamicsWorld->addCollisionObject(pc.collisionObject);
                        std::cout << "Survived" << std::endl;
                    });
            world.observer<PhysicsComponent>("Physics Body removed")
                    .event(flecs::OnRemove)
                    .each([dynamicsWorld](PhysicsComponent &pc) {
                        std::cout << "Removed physics component" << std::endl;
                        dynamicsWorld->removeCollisionObject(pc.collisionObject);
                    });
            world.system<PositionComponent>("Movement System")
                    .without<PhysicsComponent>()
                    .iter([](flecs::iter it, PositionComponent *pc) {
                        for (auto i: it) {
                            pc[i].update(it.delta_time());
                        }
                    });
        }
    };
}

#endif //CFORGESANDBOX_PHYSICSSYSTEM_H
