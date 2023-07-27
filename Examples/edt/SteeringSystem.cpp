#include <iostream>
#include "SteeringSystem.h"
#include "crossforge/Math/CForgeMath.h"
#include "flecs.h"
#include <iostream>
#include "crossforge/Graphics/SceneGraph/SGNTransformation.h"
#include "AIComponent.h"
#include "SteeringSystem.h"
#include "Obstacle.h"
#include "SteeringComponent.h"
#include "crossforge/Graphics/SceneGraph/SGNGeometry.h"


namespace CForge {
        void SteeringSystem::addSteeringSystem(flecs::world& world) {
            world.system<SGNTransformation, AIComponent, SteeringComponent, SGNGeometry>("SteeringSystem")
                .iter([&world](flecs::iter it, SGNTransformation* p, AIComponent* ai, SteeringComponent* sc, SGNGeometry* geo) {
                for (int i : it) {
                    SteeringSystem::processEntity(it.delta_time(), ai[i], p[i], world);
                }
                    });
        }

        void SteeringSystem::processEntity(float dt, AIComponent& ai, SGNTransformation& p, SteeringComponent& sc, SGNGeometry& geo, flecs::world& world) {
            std::vector<std::tuple<Eigen::Vector3f,float>> obstacles;
            world.filter<SGNTransformation, Obstacle, SGNGeometry>()
                .each([&obstacles](const SGNTransformation& t, Obstacle o,SGNGeometry geo) {
                const Eigen::Matrix4f R = CForgeMath::rotationMatrix(t.rotation());
                const Eigen::Matrix4f T = CForgeMath::translationMatrix(t.translation());
                const Eigen::Matrix4f S = CForgeMath::scaleMatrix(t.scale());
                const Eigen::Matrix4f ModelMat = T * R * S;
                float obstalceRadius = geo.actor().getAABBradius(ModelMat);
                obstacles.push_back({ t.translation(), obstalceRadius });

                    });
            std::sort(obstacles.begin(), obstacles.end(),
                [&p](auto v1, auto v2) { return (p.translation() - v1).norm() < (p.translation() - v2).norm(); });
            if (!ai.path.empty()) {
                Eigen::Vector3f target = ai.path.front();
                if (SteeringSystem::arrivedAtWayPoint(p.translation(), target)) {
                    ai.path.pop();
                }
                for (auto pos : obstacles) {
                    if (SteeringSystem::obstacleAvoidance(p, world, target, get<0>(pos), get<1>(pos),1,0.5f) {
                        break;
                    }
                }
                SteeringSystem::seekingBehavior(dt, target, p);
            }
        }
        bool SteeringSystem::obstacleIsInPath(SGNTransformation& p, Eigen::Vector3f& target,
            Eigen::Vector3f& obstaclePosition, float obstacleRadius,
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

        bool SteeringSystem::obstacleAvoidance(SGNTransformation& p, flecs::world& world, Eigen::Vector3f& target,
            Eigen::Vector3f& obstacle, float obstacleRadius, float roboterRadius, float securityDistance) {
            if (obstacleIsInPath(p, target, obstacle, obstacleRadius, roboterRadius)) {

                auto diff = Eigen::Vector3f(-(obstacle.z() - p.translation().z()), 0,
                    obstacle.x() - p.translation().x()).normalized() *
                    (obstacleRadius + roboterRadius + securityDistance);
                auto t1 = obstacle + diff;
                auto t2 = obstacle - diff;

                if ((t1 - target).norm() < (t2 - target).norm()) {
                    target = t1;
                }
                else {
                    target = t2;
                }
                return true;
            }
            return false;
        }

        bool SteeringSystem::arrivedAtWayPoint(Eigen::Vector3f position, Eigen::Vector3f target) {
            return (position - target).norm() < 2;
        }

        void SteeringSystem::seekingBehavior(float dt, Eigen::Vector3f targetPosition, SGNTransformation& p) {

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
