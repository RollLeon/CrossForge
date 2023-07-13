#ifndef STEERINGCOMPONENT_H
#define STEERINGCOMPONENT_H

#include <flecs.h>
#include "crossforge/Graphics/SceneGraph/SGNTransformation.h"

namespace CForge {

    class SteeringComponent {
    public:
        static bool obstacleIsInPath(SGNTransformation& p,Eigen::Vector3f& target, Eigen::Vector3f& obstaclePosition, float obstacleRadius, float robotRadius);

        static bool obstacleAvoidance(SGNTransformation& p, flecs::world& world, Eigen::Vector3f& target, Eigen::Vector3f& obstacle);

        static bool arrivedAtWayPoint(Eigen::Vector3f position, Eigen::Vector3f target);

        static void seekingBehavior(float dt, Eigen::Vector3f targetPosition, SGNTransformation& p);
    };

} // CForge

#endif // STEERINGCOMPONENT_H
