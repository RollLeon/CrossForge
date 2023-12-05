#ifndef CFORGESANDBOX_SYSTEMS_H
#define CFORGESANDBOX_SYSTEMS_H

#include <flecs.h>
#include <iostream>
#include "Components.h"

namespace CForge {

    class Systems {
    public:

        static void addSimpleSystems(flecs::world &world) {
            world.system<AIComponent>("AISystem")
                    .iter([&world](flecs::iter it, AIComponent *ai) {
                        for (int i: it) {
                            ai[i].tree.tickExactlyOnce();
                        }
                    });
            float waterDecreaseRate = 0.1;
            world.system<PlantComponent>("PlantSystem")
                    .iter([waterDecreaseRate](flecs::iter it, PlantComponent *p) {
                        for (int i: it) {
                            if (p[i].waterLevel > 0) {
                                p[i].waterLevel -= waterDecreaseRate * it.delta_time();
                            } else {
                                p[i].waterLevel = 0; // Ensure the water level doesn't go negative
                            }
                        }
                    });
        }
    };

} // CForge

#endif //CFORGESANDBOX_SYSTEMS_H