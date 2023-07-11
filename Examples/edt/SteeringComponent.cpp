#include "SteeringComponent.h"
#include "CForgeMath.h" // Falls erforderlich

namespace CForge {

    bool SteeringComponent::obstacleIsInPath(SGNTransformation& p, Vector3f& obstaclePosition, float obstacleRadius, float robotRadius) {
        obstaclePosition = Vector3f(0, 0, 0);
        // Projektion des Mittelpunkts des Objekts auf den Bewegungsvektor
        Vector3f motionVector = p.translationDelta().normalized();
        Vector3f objectToRobot = obstaclePosition - p.translation();
        Vector3f projectedPoint = motionVector * motionVector.dot(objectToRobot);

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

    void SteeringComponent::obstacleAvoidance(SGNTransformation& p, flecs::world& world, Vector3f& target) {
        Vector3f obstacle = Vector3f();
        float obstacleRadius = 1;
        float roboterRadius = 1;
        float securityDistance = 0.5;
        if (obstacleIsInPath(p, obstacle, obstacleRadius, roboterRadius)) {
            target = obstacle + Vector3f(-(obstacle.z() - p.translation().z()), 0,
                obstacle.x() - p.translation().x()).normalized() *
                (obstacleRadius + roboterRadius + securityDistance);
        }
    }

    bool SteeringComponent::arrivedAtWayPoint(Vector3f position, Vector3f target) {
        return (position - target).norm() < 2;
    }

    void SteeringComponent::seekingBehavior(float dt, Vector3f targetPosition, SGNTransformation& p) {
        float mass = 500.0;
        float max_force = 0.6f;
        float max_speed = 0.05f;
        Vector3f desired_velocity = (targetPosition - p.translation() - p.translationDelta());
        Vector3f steering_force = CForgeMath::maxLength(desired_velocity, max_force);
        Vector3f uncapped_velocity = p.translationDelta() + steering_force / mass;
        p.translationDelta(CForgeMath::maxLength(uncapped_velocity, max_speed));
        Quaternionf rotation = Quaternionf::FromTwoVectors(Vector3f::UnitX(), p.translationDelta().normalized());
        p.rotation(rotation);
        p.update(dt);
    }

} // CForge
