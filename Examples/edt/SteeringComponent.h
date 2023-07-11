#ifndef STEERINGCOMPONENT_H
#define STEERINGCOMPONENT_H

#include <flecs.h>
#include "crossforge/Graphics/SceneGraph/SGNTransformation.h"

namespace CForge {

    class SteeringComponent {
    public:
        static bool obstacleIsInPath(SGNTransformation& p, Vector3f& obstaclePosition, float obstacleRadius, float robotRadius);

        static void obstacleAvoidance(SGNTransformation& p, flecs::world& world, Vector3f& target);

        static bool arrivedAtWayPoint(Vector3f position, Vector3f target);

        static void seekingBehavior(float dt, Vector3f targetPosition, SGNTransformation& p);
    };

} // CForge

#endif // STEERINGCOMPONENT_H
