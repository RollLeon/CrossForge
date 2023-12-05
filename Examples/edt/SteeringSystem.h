#ifndef STEERINGSYSTEM_H
#define STEERINGSYSTEM_H

#include <flecs.h>
#include "Components.h"
#include "crossforge/Graphics/SceneGraph/SGNGeometry.h"
#include "crossforge/Math/CForgeMath.h"

namespace CForge {

    class SteeringSystem {
    public:
        static void addSteeringSystem(flecs::world &world) {
            world.system<PositionComponent, PathComponent, SteeringComponent, GeometryComponent>("SteeringSystem")
                    .iter([&world](flecs::iter it, PositionComponent *p, PathComponent *ai, SteeringComponent *sc,
                                   GeometryComponent *geo) {
                        for (int i: it) {
                            SteeringSystem::processEntity(it.delta_time(), ai[i], p[i], sc[i], geo[i], world);
                        }
                    });
        }

        static void
        processEntity(float dt, PathComponent &ai, PositionComponent &p, SteeringComponent &sc, GeometryComponent &geo,
                      flecs::world &world) {
            float robotRadius = 0;
                // geo.actor->boundingVolume().boundingSphere().radius() * p.scale().x();
            std::vector<std::tuple<Eigen::Vector3f, float>> obstacles;
            world.filter<PositionComponent, ObstacleComponent, GeometryComponent>()
                    .each([&obstacles, p](const PositionComponent &t, ObstacleComponent o, GeometryComponent geo) {
                        float obstalceRadius = geo.actor->boundingVolume().boundingSphere().radius() * t.scale().x();
                        if ((p.translation()-t.translation()).norm() > 0.1)
                {
                    obstacles.emplace_back(t.translation(), obstalceRadius);
                }

                    });
            std::sort(obstacles.begin(), obstacles.end(),
                      [&p](auto v1, auto v2) {
                          return (p.translation() - std::get<0>(v1)).norm() <
                                 (p.translation() - std::get<0>(v2)).norm();
                      });
            if (!ai.path.empty()) {
                Eigen::Vector3f target = ai.path.front();
                //Adding the correct Radius for a plant
                if (SteeringSystem::arrivedAtWayPoint(p.translation(), target, robotRadius, 0, sc.securityDistance)) {
                    ai.path.pop();
                }
                for (auto pos: obstacles) {
                    if (SteeringSystem::obstacleAvoidance(p, world, target, std::get<0>(pos), std::get<1>(pos),
                                                          robotRadius,
                                                          sc.securityDistance)) {
                        break;
                    }
                }
                SteeringSystem::seekingBehavior(dt, target, p, sc);
            } else {
                p.translationDelta(Eigen::Vector3f(0, 0, 0));
            }
        }

        static bool obstacleIsInPath(PositionComponent &p, Eigen::Vector3f &target, Eigen::Vector3f &obstaclePosition,
                                     float obstacleRadius, float robotRadius, float securityDistance) {
            // Projektion des Mittelpunkts des Objekts auf den Bewegungsvektor
            Eigen::Vector3f motionVector = (target - p.translation()).normalized();
            Eigen::Vector3f objectToRobot = obstaclePosition - p.translation();
            float d = motionVector.dot(objectToRobot);
            if (d < 0)return false;
            Eigen::Vector3f projectedPoint = motionVector * d + p.translation();

            // Berechne den Abstand zwischen dem projizierten Punkt und dem Mittelpunkt des Objekts
            Eigen::Vector3f distancev = obstaclePosition - projectedPoint;
            float distance = distancev.norm();

            // Überprüfe, ob der Abstand größer ist als die Summe der Radien
            float totalRadius = obstacleRadius + robotRadius + securityDistance;
            return distance < totalRadius;
            // return p.translationDelta().dot(-p.translation()) > 0;
        }

        static bool
        obstacleAvoidance(PositionComponent &p, flecs::world &world, Eigen::Vector3f &target, Eigen::Vector3f &obstacle,
                          float obstacleRadius, float roboterRadius, float securityDistance) {
            if (obstacleIsInPath(p, target, obstacle, obstacleRadius, roboterRadius, securityDistance)) {

                auto diff = Eigen::Vector3f(-(obstacle.z() - p.translation().z()), 0,
                                            obstacle.x() - p.translation().x()).normalized() *
                            (obstacleRadius + roboterRadius + securityDistance);
                auto t1 = obstacle + diff;
                auto t2 = obstacle - diff;

                if ((t1 - target).norm() < (t2 - target).norm()) {
                    target = t1;
                } else {
                    target = t2;
                }
                return true;
            }
            return false;
        }

        static bool
        arrivedAtWayPoint(Eigen::Vector3f position, Eigen::Vector3f target, float RobotRadius, float ObstacleRadius,
                          float securityDistance) {
            return (position - target).norm() < (RobotRadius + ObstacleRadius + securityDistance);
        }

        static void
        seekingBehavior(float dt, Eigen::Vector3f targetPosition, PositionComponent &p, SteeringComponent &sc) {
            Eigen::Vector3f toTarget = targetPosition - p.translation();
            if (p.translationDelta().dot(toTarget) < 0) {
                Eigen::Vector3f velocityNormal = Eigen::Vector3f(-p.translationDelta().z(), p.translationDelta().y(),
                                                                 p.translationDelta().x());
                float multiplier = toTarget.dot(velocityNormal) > 0 ? 1 : -1;
                targetPosition = velocityNormal.normalized() * toTarget.norm() * multiplier;
                targetPosition = targetPosition + p.translation();
            }
            Eigen::Vector3f desired_velocity = (targetPosition - p.translation() - p.translationDelta());
            Eigen::Vector3f steering_force = CForgeMath::maxLength(desired_velocity, sc.max_force);
            Eigen::Vector3f uncapped_velocity = p.translationDelta() + steering_force / sc.mass;
            p.translationDelta(CForgeMath::maxLength(uncapped_velocity, sc.max_speed));
            if (p.translationDelta().norm() > 0.001) {

                float lerpFactor = 1.0 - pow(0.05, dt);

                Eigen::Quaternionf currentRotation = p.rotation();

                Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitX(),
                                                                                 Eigen::Vector3f(
                                                                                         p.translationDelta().x(), 0,
                                                                                         p.translationDelta().z()).normalized());

                Eigen::Quaternionf interpolatedRotation = currentRotation.slerp(lerpFactor, rotation);

                p.rotation(interpolatedRotation);
            }
        }
    };

} // CForge

#endif // STEERINGSYSTEM_H
