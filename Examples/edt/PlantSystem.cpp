#include "PlantSystem.h"
#include "PathComponent.h"


namespace CForge {

    // Define the rate at which the water level decreases (e.g., 1 unit per second)
    float waterDecreaseRate = 0.1;
    float waterIncreaseRate = 1.0;



    void PlantSystem::reduceWaterLevel(PlantComponent &p) {
        // Check if the water level is greater than the decrease rate
        if (p.waterLevel > waterDecreaseRate) {
            p.waterLevel -= waterDecreaseRate;
        }
        else {
            p.waterLevel = 0; // Ensure the water level doesn't go negative
        }
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
