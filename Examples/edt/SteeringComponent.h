#ifndef STEERINGCOMPONENT_H
#define STEERINGCOMPONENT_H

#include <flecs.h>
#include "crossforge/Graphics/SceneGraph/SGNTransformation.h"

namespace CForge {

    class SteeringComponent {
    public:
        float max_force;
        float max_speed;
        float mass;
    };

} // CForge

#endif // STEERINGCOMPONENT_H
