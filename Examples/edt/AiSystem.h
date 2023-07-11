//
// Created by Linus on 29.06.23.
//

#ifndef CFORGESANDBOX_AISYSTEM_H
#define CFORGESANDBOX_AISYSTEM_H

#include <flecs.h>
#include <iostream>
#include "crossforge/Graphics/SceneGraph/SGNTransformation.h"
#include "AIComponent.h"
#include "SteeringComponent.h"

namespace CForge {

    class AiSystem {
    public:
        static void addAiSystem(flecs::world &world) {
            world.system<SGNTransformation, AIComponent>("AISystem")
                    .iter([](flecs::iter it, SGNTransformation *p, AIComponent *ai) {
                        for (int i: it) {
                            AiSystem::processEntity(it.delta_time(), ai[i], p[i], it.world());
                        }
                    });
        }

    protected:
        static void processEntity(float dt, AIComponent &ai, SGNTransformation &p, flecs::world world) {

            if (!ai.path.empty()) {
                Vector3f target = ai.path.front();
                if (arrivedAtWayPoint(p.translation(), target)) {
                    ai.path.pop();
                }
                obstacleAvoidance(p, world, target);
                seekingBehavior(dt, target, p);
            } else {
            }
            if (ai.path.empty()) {
                addRandomTarget(ai.path);
            }
        }

        static bool obstacleIsInPath(SGNTransformation& p, Vector3f& obstaclePosition, float obstacleRadius, float robotRadius) {
            return SteeringComponent::obstacleIsInPath(p, obstaclePosition, obstacleRadius, robotRadius);
        }

        static void obstacleAvoidance(SGNTransformation& p, flecs::world& world, Vector3f& target) {
            SteeringComponent::obstacleAvoidance(p, world, target);
        }


        static bool arrivedAtWayPoint(Eigen::Vector3f position, Vector3f target) {
            return SteeringComponent::arrivedAtWayPoint(position, target);
        }


        static void addRandomTarget(std::queue<Eigen::Vector3f> &vecQueue) {
            Eigen::Vector3f targetPosition = Vector3f();
            targetPosition.setRandom();
            targetPosition.y() = 0;
            targetPosition *= 20;
            vecQueue.push(targetPosition);
        }

        static void seekingBehavior(float dt, Eigen::Vector3f targetPosition, SGNTransformation& p) {
            SteeringComponent::seekingBehavior(dt, targetPosition, p);
        }


    };

} // CForge

#endif //CFORGESANDBOX_AISYSTEM_H
