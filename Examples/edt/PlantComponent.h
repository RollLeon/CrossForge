#ifndef PLANTCOMPONENT_H
#define PLANTCOMPONENT_H

#include <flecs.h>
#include "crossforge/Graphics/SceneGraph/SGNTransformation.h"

namespace CForge {

    class PlantComponent {
    public:
        float waterLevel;

        //Muss ich hier ein Maximum festlegen, damit der Gießroboter nicht endlos gießt?
        float maxWaterLevel = 10.0;
    };

} // CForge

#endif // PLANTCOMPONENT_H
