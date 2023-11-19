#ifndef CFORGESANDBOX_AICOMPONENT_H
#define CFORGESANDBOX_AICOMPONENT_H

#include <Eigen/Core>
#include "behaviortree_cpp/bt_factory.h"

namespace CForge {
    class AIComponent {
    public:
        BT::Tree tree;
    };
}
#endif //CFORGESANDBOX_AICOMPONENT_H
