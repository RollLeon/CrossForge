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

    void PlantSystem::increaseWaterLevel(PlantComponent &p) {
        // You can implement this function to increase the water level by a specific amount
        if (p.waterLevel + waterIncreaseRate < p.maxWaterLevel) {
            p.waterLevel += waterIncreaseRate;
        } else {
            p.waterLevel = p.maxWaterLevel;
        }
    }


} // CForge
