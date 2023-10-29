//
// Created by Linus on 29.06.23.
//

#ifndef CFORGESANDBOX_PATHSYSTEM_H
#define CFORGESANDBOX_PATHSYSTEM_H

#include <flecs.h>
#include <iostream>
#include "crossforge/Graphics/SceneGraph/SGNTransformation.h"
#include "PathComponent.h"
#include "SteeringSystem.h"
#include "Obstacle.h"

namespace CForge {

    class PathSystem {
    public:
        static void addPathSystem(flecs::world &world) {
            world.system<SGNTransformation, PathComponent>("PathSystem")
                    .iter([&world](flecs::iter it, SGNTransformation *p, AIComponent *ai) {
                        for (int i: it) {
                            PathSystem::processEntity(it.delta_time(), ai[i], p[i], world);
                        }
                    });
        }

    protected:
        static void processEntity(float dt, PathComponent &ai, SGNTransformation &p, flecs::world &world) {
            
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

#endif //CFORGESANDBOX_PATHSYSTEM_H
