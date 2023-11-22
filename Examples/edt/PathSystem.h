//
// Created by Linus on 29.06.23.
//

#ifndef CFORGESANDBOX_PATHSYSTEM_H
#define CFORGESANDBOX_PATHSYSTEM_H

#include <flecs.h>
#include <iostream>
#include "crossforge/Graphics/SceneGraph/SGNTransformation.h"
#include "SteeringSystem.h"
#include "Components.h"

namespace CForge {

    class PathSystem {
    public:
        static void addPathSystem(flecs::world &world) {
            world.system<PathRequestComponent>("PathSystem")
                    .iter([&world](flecs::iter it, PathRequestComponent *p) {
                        for (auto i: it) {
                            flecs::entity e = it.entity(i);
                            auto destination = p[i].destination;
                            e.remove<PathRequestComponent>();
                            e.add<PathComponent>();
                            auto pathComponent = e.get_mut<PathComponent>();
                            pathComponent->path.push(destination);
                        }
                    });
        }

    };

} // CForge

#endif //CFORGESANDBOX_PATHSYSTEM_H
