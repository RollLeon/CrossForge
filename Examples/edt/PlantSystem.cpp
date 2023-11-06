#include "PlantSystem.h"
#include "PathComponent.h"


namespace CForge {

    // Define the rate at which the water level decreases (e.g., 1 unit per second)
    float waterDecreaseRate = 0.1;
    float waterIncreaseRate = 1.0;



    void PlantSystem::reduceWaterLevel(flecs::world& world) {
        world.query<PlantComponent>()
            .iter([&world](flecs::iter it, PlantComponent* p) {
                for (int i : it) {
                    // Check if the water level is greater than the decrease rate
                    if (p[i].waterLevel > waterDecreaseRate) {
                        p[i].waterLevel -= waterDecreaseRate;
                    }
                    else {
                        p[i].waterLevel = 0; // Ensure the water level doesn't go negative
                    }
                }
            });
    }

    void PlantSystem::increaseWaterLevel(PlantComponent &p) {
        // You can implement this function to increase the water level by a specific amount
        if (p.waterLevel + waterIncreaseRate< p.maxWaterlevel){
            p.waterLevel += waterIncreaseRate;
        }
        else{
            p.waterLevel = p.maxWaterlevel;
        }
    }

    
    
    
} // CForge
