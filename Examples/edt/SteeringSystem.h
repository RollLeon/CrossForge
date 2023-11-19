#ifndef STEERINGSYSTEM_H
#define STEERINGSYSTEM_H

#include <flecs.h>
#include "PathComponent.h"
#include "SteeringComponent.h"
#include "crossforge/Graphics/SceneGraph/SGNGeometry.h"
#include "PositionComponent.h"
#include "GeometryComponent.h"

namespace CForge {

    class SteeringSystem {
    public:
        static void addSteeringSystem(flecs::world& world);

        static void processEntity(float dt, PathComponent& ai, PositionComponent& p, SteeringComponent& sc, GeometryComponent& geo, flecs::world& world);

        static bool obstacleIsInPath(PositionComponent& p,Eigen::Vector3f& target, Eigen::Vector3f& obstaclePosition, float obstacleRadius, float robotRadius, float securityDistance);

        static bool obstacleAvoidance(PositionComponent& p, flecs::world& world, Eigen::Vector3f& target, Eigen::Vector3f& obstacle, float obstacleRadius, float roboterRadius, float securityDistance);

        static bool arrivedAtWayPoint(Eigen::Vector3f position, Eigen::Vector3f target);

        static void seekingBehavior(float dt, Eigen::Vector3f targetPosition, PositionComponent& p, SteeringComponent& sc);
    };

} // CForge

#endif // STEERINGSYSTEM_H
