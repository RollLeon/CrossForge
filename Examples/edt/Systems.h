#ifndef CFORGESANDBOX_SYSTEMS_H
#define CFORGESANDBOX_SYSTEMS_H

#include <flecs.h>
#include <iostream>
#include "Components.h"
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <LinearMath/btDefaultMotionState.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <random>
#include "crossforge/Graphics/Actors/StaticActor.h"
#include "crossforge/AssetIO/SAssetIO.h"


namespace CForge {

    class Systems {
    public:

        static void addSimpleSystems(flecs::world &world, StaticActor *waterDrop) {
            world.system<AIComponent>("AISystem")
                    .iter([&world](flecs::iter it, AIComponent *ai) {
                        for (int i: it) {
                            ai[i].tree.tickExactlyOnce();
                        }
                    });

            float waterDecreaseRate = 0.1;
            world.system<PlantComponent>("PlantSystem")
                    .iter([waterDecreaseRate](flecs::iter it, PlantComponent *p) {
                        for (int i: it) {
                            if (p[i].waterLevel > 0) {
                                p[i].waterLevel -= waterDecreaseRate * it.delta_time();
                            } else {
                                p[i].waterLevel = 0; // Ensure the water level doesn't go negative
                            }
                        }
                    });

            world.system<ParticleComponent>("ParticleRemoverSystem")
                    .iter([&world](flecs::iter it, ParticleComponent *pc) {
                        for (int i: it) {
                            ParticleComponent *particleComp = it.entity(i).get_mut<ParticleComponent>();
                            if (particleComp->lifeTime > 0) {
                                particleComp->lifeTime -= it.delta_time();
                            } else {
                                it.entity(i).destruct();
                            }
                        }
                    });

            world.system<EmitterComponent, PositionComponent>("WaterEmitterSystem")
                    .iter([&world, waterDrop](flecs::iter it, EmitterComponent *em, PositionComponent *pc) {
                        for (int e: it) {
                            std::random_device rd;
                            std::mt19937 mt(rd());
                            std::uniform_real_distribution<float> dist(-1.0, 1.0);
                            for (int p = 0; p < em[e].numParticles; p++) {
                                auto entity = world.entity();
                                entity.add<PositionComponent>();
                                entity.add<GeometryComponent>();
                                entity.add<ParticleComponent>();
                                auto particle = entity.get_mut<ParticleComponent>();
                                particle->lifeTime = 2.0;
                                PositionComponent *entityPosition = entity.get_mut<PositionComponent>();
                                entityPosition->init();

                                GeometryComponent *obstacle_geom = entity.get_mut<GeometryComponent>();
                                obstacle_geom->init(waterDrop);
                                auto spawnOffset = Eigen::Vector3f(0.5 * dist(mt), 0.5 * dist(mt),
                                                                   (dist(mt) - 1) * 0.25f);

                                entityPosition->translation(
                                        pc[e].rotation() * (em[e].relativePosition + spawnOffset) +
                                        pc[e].m_Translation);
                                float spread = 0.75f;
                                auto direction = pc[e].rotation() *
                                                 Eigen::Vector3f(spread * dist(mt), -3,
                                                                 spread * (dist(mt) - 1) / 2.0f - 1);
                                entityPosition->translationDelta(direction);
                            }
                        }
                    });

        }

    };


} // CForge

#endif //CFORGESANDBOX_SYSTEMS_H