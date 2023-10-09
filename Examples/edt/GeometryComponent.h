
#ifndef GEOMETRYCOMPONENT_H
#define GEOMETRYCOMPONENT_H

#include "crossforge/Graphics/Actors/IRenderableActor.h"

namespace CForge {
    class GeometryComponent {
    public:
        IRenderableActor* actor;

        void init(IRenderableActor *pActor) {
            actor = pActor;
        }
    };

}
#endif //GEOMETRYCOMPONENT_H
