//
// Created by private on 22.06.23.
//

#include "AIComponent.h"

namespace CForge {

    AIComponent::AIComponent() {
        path = std::queue<Eigen::Vector3f>();
    }
}