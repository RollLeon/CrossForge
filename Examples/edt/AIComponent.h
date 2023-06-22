//
// Created by private on 22.06.23.
//

#ifndef CFORGESANDBOX_AICOMPONENT_H
#define CFORGESANDBOX_AICOMPONENT_H

#include <Eigen/Core>

namespace CForge {
    class AIComponent {
    public:
        Eigen::Vector3f targetPosition;

        AIComponent();
    };

}
#endif //CFORGESANDBOX_AICOMPONENT_H
