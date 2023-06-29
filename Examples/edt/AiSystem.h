//
// Created by Linus on 29.06.23.
//

#ifndef CFORGESANDBOX_AISYSTEM_H
#define CFORGESANDBOX_AISYSTEM_H

#include <flecs.h>
#include <iostream>
#include "crossforge/Graphics/SceneGraph/SGNTransformation.h"
#include "AIComponent.h"

namespace CForge {

    class AiSystem {
    public:
        static void addAiSystem(flecs::world &world) {
            world.system<SGNTransformation, AIComponent>("AISystem")
                    .iter([](flecs::iter it, SGNTransformation *p, AIComponent *ai) {
                        for (int i: it) {
                            AiSystem::processEntity(it.delta_time(), ai[i], p[i]);
                        }
                    });
        }

    protected:
        static void processEntity(float dt, AIComponent &ai, SGNTransformation &p) {

            if (!ai.path.empty()) {
                Vector3f target = ai.path.front();
                if (arrivedAtWayPoint(p.translation(), target)) {
                    ai.path.pop();
                }

                seekingBehavior(dt, target, p);
            } else {
            }
            if (ai.path.empty()) {
                addRandomTarget(ai.path);
            }
        }

        static bool arrivedAtWayPoint(Eigen::Vector3f position, Vector3f target) {
            return (position - target).norm() < 2;
        }

        static void addRandomTarget(std::queue<Eigen::Vector3f> &vecQueue) {
            Eigen::Vector3f targetPosition = Vector3f();
            targetPosition.setRandom();
            targetPosition.y() = 0;
            targetPosition *= 20;
            vecQueue.push(targetPosition);
        }

        static void seekingBehavior(float dt, Eigen::Vector3f targetPosition, SGNTransformation &p) {
            float mass = 500.0;
            float max_force = 0.3f;
            float max_speed = 0.05f;
            Eigen::Vector3f desired_velocity = (targetPosition - p.translation() - p.translationDelta());
            Eigen::Vector3f steering_force = CForgeMath::maxLength(desired_velocity, max_force);
            Eigen::Vector3f uncapped_velocity = p.translationDelta() + steering_force / mass;
            p.translationDelta(CForgeMath::maxLength(uncapped_velocity, max_speed));
            Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(Vector3f::UnitX(),
                                                                             p.translationDelta().normalized());
            p.rotation(rotation);
            p.update(dt);
        }

    };

} // CForge

#endif //CFORGESANDBOX_AISYSTEM_H
