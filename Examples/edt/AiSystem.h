//
// Created by Linus on 29.06.23.
//

#ifndef CFORGESANDBOX_AISYSTEM_H
#define CFORGESANDBOX_AISYSTEM_H

#include <flecs.h>
#include <iostream>
#include "crossforge/Graphics/SceneGraph/SGNTransformation.h"
#include "AIComponent.h"
#include "SteeringSystem.h"
#include "Obstacle.h"

namespace CForge {

    class AiSystem {
    public:
        static void addAiSystem(flecs::world &world) {
            world.system<SGNTransformation, AIComponent>("AISystem")
                    .iter([&world](flecs::iter it, SGNTransformation *p, AIComponent *ai) {
                        for (int i: it) {
                            AiSystem::processEntity(it.delta_time(), ai[i], p[i], world);
                        }
                    });
        }

    protected:
        static void processEntity(float dt, AIComponent &ai, SGNTransformation &p, flecs::world &world) {
            std::vector<Vector3f> obstacles;
            world.filter<SGNTransformation, Obstacle>()
                    .each([&obstacles](const SGNTransformation &t, Obstacle o) {
                        obstacles.push_back(t.translation());
                    });
            std::sort(obstacles.begin(), obstacles.end(),
                      [&p](auto v1, auto v2) { return (p.translation() - v1).norm() < (p.translation() - v2).norm(); });
            if (!ai.path.empty()) {
                Eigen::Vector3f target = ai.path.front();
                if (arrivedAtWayPoint(p.translation(), target)) {
                    ai.path.pop();
                }
                for (auto pos: obstacles) {
                   if(obstacleAvoidance(p, world, target, pos)){
                       break;
                   }
                }
                seekingBehavior(dt, target, p);
            } else {
            }
            if (ai.path.empty()) {
                addRandomTarget(ai.path);
            }
        }

        static bool obstacleAvoidance(SGNTransformation &p, flecs::world &world, Eigen::Vector3f &target,
                                      Eigen::Vector3f &obstaclepos) {
           return SteeringSystem::obstacleAvoidance(p, world, target, obstaclepos); 
        }


        static bool arrivedAtWayPoint(Eigen::Vector3f position, Eigen::Vector3f target) {
            return SteeringSystem::arrivedAtWayPoint(position, target);
        }


        static void addRandomTarget(std::queue<Eigen::Vector3f> &vecQueue) {
            Eigen::Vector3f targetPosition = Eigen::Vector3f();
            targetPosition.setRandom();
            targetPosition.y() = 0;
            targetPosition *= 20;
            vecQueue.push(targetPosition);
        }

        static void seekingBehavior(float dt, Eigen::Vector3f targetPosition, SGNTransformation &p) {
            SteeringSystem::seekingBehavior(dt, targetPosition, p);
        }


    };

} // CForge

#endif //CFORGESANDBOX_AISYSTEM_H
