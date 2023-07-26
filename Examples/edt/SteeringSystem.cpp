#include <iostream>
#include "SteeringSystem.h"
#include "crossforge/Math/CForgeMath.h"
#include "flecs.h"

namespace CForge {

    bool SteeringSystem::obstacleIsInPath(SGNTransformation &p, Eigen::Vector3f &target,
                                             Eigen::Vector3f &obstaclePosition, float obstacleRadius,
                                             float robotRadius) {
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
        float securityDistance = 0.5;
        float totalRadius = obstacleRadius + robotRadius + securityDistance;
        return distance < totalRadius;
        // return p.translationDelta().dot(-p.translation()) > 0;
    }

    bool SteeringSystem::obstacleAvoidance(SGNTransformation &p, flecs::world &world, Eigen::Vector3f &target,
                                              Eigen::Vector3f &obstacle) {
        float obstacleRadius = 1;
        float roboterRadius = 1;
        float securityDistance = 0.5;
        if (obstacleIsInPath(p, target, obstacle, obstacleRadius, roboterRadius)) {

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

    bool SteeringSystem::arrivedAtWayPoint(Eigen::Vector3f position, Eigen::Vector3f target) {
        return (position - target).norm() < 2;
    }

    void SteeringSystem::seekingBehavior(float dt, Eigen::Vector3f targetPosition, SGNTransformation &p) {
        
        float mass = 500.0;
        float max_force = 0.6f;
        float max_speed = 0.05f;
        Eigen::Vector3f desired_velocity = (targetPosition - p.translation() - p.translationDelta());
        Eigen::Vector3f steering_force = CForgeMath::maxLength(desired_velocity, max_force);
        Eigen::Vector3f uncapped_velocity = p.translationDelta() + steering_force / mass;
        p.translationDelta(CForgeMath::maxLength(uncapped_velocity, max_speed));
        Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitX(),
                                                                         p.translationDelta().normalized());
        p.rotation(rotation);
        p.update(dt);
    }

} // CForge
