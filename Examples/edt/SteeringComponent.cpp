#include "SteeringComponent.h"
#include "crossforge/Math/CForgeMath.h"
namespace CForge {

    bool SteeringComponent::obstacleIsInPath(SGNTransformation& p, Eigen::Vector3f& obstaclePosition, float obstacleRadius, float robotRadius) {
        obstaclePosition = Eigen::Vector3f(0, 0, 0);
        // Projektion des Mittelpunkts des Objekts auf den Bewegungsvektor
        Eigen::Vector3f motionVector = p.translationDelta().normalized();
        Eigen::Vector3f objectToRobot = obstaclePosition - p.translation();
        Eigen::Vector3f projectedPoint = motionVector * motionVector.dot(objectToRobot);

        float dotProduct = motionVector.dot(objectToRobot); // Skalarprodukt der beiden Vektoren
        float magnitudeProduct = motionVector.norm() * objectToRobot.norm(); // Produkt der Magnituden der beiden Vektoren

        // Berechne den Arcustangens des Verhältnisses der beiden Produkte
        float angle = std::acos(dotProduct / magnitudeProduct);
        if (angle > 90)
        {
            return false;
        }

        // Berechne den Abstand zwischen dem projizierten Punkt und dem Mittelpunkt des Objekts
        float distance = (obstaclePosition - projectedPoint).norm();

        // Überprüfe, ob der Abstand größer ist als die Summe der Radien
        float totalRadius = obstacleRadius + robotRadius;
        return distance > totalRadius;
    }

    void SteeringComponent::obstacleAvoidance(SGNTransformation& p, flecs::world& world, Eigen::Vector3f& target) {
        Eigen::Vector3f obstacle = Eigen::Vector3f();
        float obstacleRadius = 1;
        float roboterRadius = 1;
        float securityDistance = 0.5;
        if (obstacleIsInPath(p, obstacle, obstacleRadius, roboterRadius)) {
            target = obstacle + Eigen::Vector3f(-(obstacle.z() - p.translation().z()), 0,
                obstacle.x() - p.translation().x()).normalized() *
                (obstacleRadius + roboterRadius + securityDistance);
        }
    }

    bool SteeringComponent::arrivedAtWayPoint(Eigen::Vector3f position, Eigen::Vector3f target) {
        return (position - target).norm() < 2;
    }

    void SteeringComponent::seekingBehavior(float dt, Eigen::Vector3f targetPosition, SGNTransformation& p) {
        float mass = 500.0;
        float max_force = 0.6f;
        float max_speed = 0.05f;
        Eigen::Vector3f desired_velocity = (targetPosition - p.translation() - p.translationDelta());
        Eigen::Vector3f steering_force = CForgeMath::maxLength(desired_velocity, max_force);
        Eigen::Vector3f uncapped_velocity = p.translationDelta() + steering_force / mass;
        p.translationDelta(CForgeMath::maxLength(uncapped_velocity, max_speed));
        Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitX(), p.translationDelta().normalized());
        p.rotation(rotation);
        p.update(dt);
    }

} // CForge
