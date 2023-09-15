#ifndef CFORGESANDBOX_RENDERINGSYSTEM_H
#define CFORGESANDBOX_RENDERINGSYSTEM_H

#include <flecs.h>
#include <iostream>
#include "crossforge/Graphics/SceneGraph/SGNTransformation.h"
#include "AIComponent.h"
#include "SteeringSystem.h"
#include "PositionComponent.h"
#include "GeometryComponent.h"
#include "Obstacle.h"

namespace CForge {

    class RenderingSystem {
    public:
        static void addRenderingSystem(flecs::world& world, RenderDevice* pRDev) {
            world.system<PositionComponent, GeometryComponent>("RenderingSystem")
                .iter([&world, pRDev](flecs::iter it, PositionComponent* p, GeometryComponent* geo) {
                    for (int i : it) {
                        RenderingSystem::processEntity(p[i], geo[i], world, pRDev);
                    }
                });
        }

    protected:
		static void processEntity(const PositionComponent& p, const GeometryComponent& geo, flecs::world& world, RenderDevice* pRDev) {

            pRDev->requestRendering(geo.actor, p.m_Rotation, p.m_Translation, p.m_Scale);
		}


    };

} // CForge

#endif //CFORGESANDBOX_RENDERINGSYSTEM_H