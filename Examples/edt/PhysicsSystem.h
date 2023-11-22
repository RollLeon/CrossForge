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
#include "Components.h"

namespace CForge {
    class PhysicsSystem {
    public:
        static void addPhysicsSystem(flecs::world &world) {
            btDefaultCollisionConfiguration *collisionConfiguration = new btDefaultCollisionConfiguration();
            btCollisionDispatcher *dispatcher = new btCollisionDispatcher(collisionConfiguration);
            btBroadphaseInterface *overlappingPairCache = new btDbvtBroadphase();
            btSequentialImpulseConstraintSolver *solver = new btSequentialImpulseConstraintSolver;

            auto dynamicsWorld = std::make_shared<btDiscreteDynamicsWorld>(dispatcher, overlappingPairCache,
                                                                           solver, collisionConfiguration);
            dynamicsWorld->setGravity(btVector3(0, -1, 0));
            world.system<PhysicsComponent, PositionComponent>("PhysicsSystem")
                    .iter([dynamicsWorld](flecs::iter it, PhysicsComponent *ps, PositionComponent *pc) {
                        for (size_t i = 0; i < it.count(); i++) {
                            auto pos = pc[i].translation();
                            auto vel = pc[i].translationDelta();
                            Eigen::Quaternionf rot = pc[i].rotation();
                            ps[i].collisionObject->activate();
                            ps[i].collisionObject->getWorldTransform().setOrigin(btVector3(pos.x(), pos.y(), pos.z()));
                            ps[i].collisionObject->setLinearVelocity(btVector3(vel.x(), vel.y(), vel.z()));
                            ps[i].collisionObject->getWorldTransform().setRotation(
                                    btQuaternion(rot.x(), rot.y(), rot.z(), rot.w()));
                        }
                        dynamicsWorld->stepSimulation(it.delta_time(), 1);
                        for (size_t i = 0; i < it.count(); i++) {

                            auto rot = ps[i].collisionObject->getWorldTransform().getRotation();
                            auto pos = ps[i].collisionObject->getWorldTransform().getOrigin();
                            auto vel = ps[i].collisionObject->getLinearVelocity();
                            pc[i].rotation(Eigen::Quaternionf(rot.w(), rot.x(), rot.y(), rot.z()));
                            pc[i].translation(Eigen::Vector3f(pos.x(), pos.y(), pos.z()));
                            pc[i].translationDelta(Eigen::Vector3f(vel.x(), vel.y(), vel.z()));
                        }
                    });
            world.observer<PhysicsComponent>("Physics Body Add Observer")
                    .event(flecs::OnSet)
                    .iter([dynamicsWorld](flecs::iter it, PhysicsComponent *pc) {
                        for (auto i: it) {
                            dynamicsWorld->addRigidBody(pc[i].collisionObject.get());
                        }
                    });
            world.observer<PhysicsComponent>("Physics Body removed")
                    .event(flecs::UnSet)
                    .each([dynamicsWorld](PhysicsComponent &pc) {
                        dynamicsWorld->removeCollisionObject(pc.collisionObject.get());
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
