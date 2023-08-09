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
            
            if (ai.path.empty()) {
                addRandomTarget(ai.path);
            }
        }

        

        static void addRandomTarget(std::queue<Eigen::Vector3f> &vecQueue) {
            Eigen::Vector3f targetPosition = Eigen::Vector3f();
            targetPosition.setRandom();
            targetPosition.y() = 0;
            targetPosition *= 20;
            vecQueue.push(targetPosition);
        }

        


    };

} // CForge

#endif //CFORGESANDBOX_AISYSTEM_H
