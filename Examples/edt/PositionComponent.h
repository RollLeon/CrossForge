
#ifndef POSITIONCOMPONENT_H
#define POSITIONCOMPONENT_H

#include <Eigen/Core>

namespace CForge {
    class PositionComponent {
    public:
		Eigen::Vector3f m_Translation;
		Eigen::Quaternionf m_Rotation;
		Eigen::Vector3f m_Scale;
		Eigen::Vector3f m_TranslationDelta;
		Eigen::Quaternionf m_RotationDelta;
		Eigen::Vector3f m_ScaleDelta; 
    };

}
#endif //POSITIONCOMPONENT_H
