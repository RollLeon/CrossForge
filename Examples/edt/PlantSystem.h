#ifndef PLANTSYSTEM_H
#define PLANTSYSTEM_H

#include "PlantComponent.h"

namespace CForge {

    class PlantSystem {
    public:
        static void redeuceWaterLevel();

        static void increaseWaterLevel();
    };

} // CForge

#endif // STEERINGSYSTEM_H
