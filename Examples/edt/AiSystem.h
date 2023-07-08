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

        /*static bool obstacleIsInPath(SGNTransformation& p, flecs::world& world, Vector3f& obstaclePosition) {
            obstaclePosition = Vector3f(0, 0, 0);
            return p.translationDelta().dot(-p.translation()) > 0;
        }*/

        static bool obstacleIsInPath(SGNTransformation& p, Vector3f& obstaclePosition, float obstacleRadius, float robotRadius) {
            obstaclePosition = Vector3f(0, 0, 0);
            // Projektion des Mittelpunkts des Objekts auf den Bewegungsvektor
            Vector3f motionVector = p.translationDelta().normalized();
            Vector3f objectToRobot =  obstaclePosition - p.translation();
            Vector3f projectedPoint = motionVector.dot(objectToRobot);

            // Berechne den Abstand zwischen dem projizierten Punkt und dem Mittelpunkt des Objekts
            float distance = (obstaclePosition - projectedPoint).norm();


            // Überprüfe, ob der Abstand größer ist als die Summe der Radien
            float totalRadius = obstacleRadius + robotRadius;
            return distance < totalRadius;
        }






        static void obstacleAvoidance(SGNTransformation &p, flecs::world &world, Vector3f &target) {
            Vector3f obstacle;
            float obstacleRadius = 2;
            float roboterRadius = 1;
            float securityDistance = 0.5;
            if (obstacleIsInPath(p, obstacle, obstacleRadius, roboterRadius)) {
                
                target = obstacle + Vector3f(-(obstacle.z() - p.translation().z()), 0,
                                             obstacle.x() - p.translation().x()).normalized() *
                                    (obstacleRadius + roboterRadius + securityDistance);
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
            float max_force = 0.6f;
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
