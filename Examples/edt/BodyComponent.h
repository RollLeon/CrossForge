#ifndef CFORGESANDBOX_BODYCOMPONENT_H
#define CFORGESANDBOX_BODYCOMPONENT_H

namespace CForge {
    class BodyComponent {
    public:
        float radius;
        float securityDistance;
        float mass;
        float max_force;
        float max_speed;
        BodyComponent();
    };

}

#endif //CFORGESANDBOX_BODYCOMPONENT_H
