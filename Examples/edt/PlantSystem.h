#ifndef PLANTSYSTEM_H
#define PLANTSYSTEM_H

#include "PlantComponent.h"
#include <flecs.h>

namespace CForge {

    class PlantSystem {
    public:
        
        static void reduceWaterLevel(flecs::world& world);

        static void increaseWaterLevel(PlantComponent& p);
    };

} // CForge

#endif // STEERINGSYSTEM_H
