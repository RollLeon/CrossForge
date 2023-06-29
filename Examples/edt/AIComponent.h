//
// Created by private on 22.06.23.
//

#ifndef CFORGESANDBOX_AICOMPONENT_H
#define CFORGESANDBOX_AICOMPONENT_H

#include <Eigen/Core>
#include <queue>

namespace CForge {
    class AIComponent {
    public:
        std::queue<Eigen::Vector3f> path;
        AIComponent();
    };

}
#endif //CFORGESANDBOX_AICOMPONENT_H
