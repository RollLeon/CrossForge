#ifndef CFORGESANDBOX_PATHCOMPONENT_H
#define CFORGESANDBOX_PATHCOMPONENT_H

#include <Eigen/Core>
#include <queue>

namespace CForge {
    class PathComponent {
    public:
        std::queue<Eigen::Vector3f> path;
        PathComponent() {
            path = std::queue<Eigen::Vector3f>();
        };
    };
}
#endif //CFORGESANDBOX_PATHCOMPONENT_H
