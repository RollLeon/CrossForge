#include <iostream>
#include "PlantSystem.h"
#include "PathComponent.h"


namespace CForge {

    // Define the rate at which the water level decreases 
    float waterDecreaseRate = 0.1;
    float waterIncreaseRate = 1.0;


    void PlantSystem::reduceWaterLevel(flecs::world &world) {
        world.query<PlantComponent>()
                .iter([&world](flecs::iter it, PlantComponent *p) {
                    for (int i: it) {
                        if (p[i].waterLevel > 0) {
                            p[i].waterLevel -= waterDecreaseRate * it.delta_time();
                        } else {
                            p[i].waterLevel = 0; // Ensure the water level doesn't go negative
                        }
                    }
                });
    }

    void PlantSystem::increaseWaterLevel(float dt, PlantComponent &p) {
        // You can implement this function to increase the water level by a specific amount
        if (p.waterLevel + waterIncreaseRate * dt < p.maxWaterLevel) {
            p.waterLevel += waterIncreaseRate * dt;
            std::cout << "increased by" << (waterIncreaseRate * dt) << std::endl;
        } else {
            p.waterLevel = p.maxWaterLevel;
        }
    }


} // CForge
