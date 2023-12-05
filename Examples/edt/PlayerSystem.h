//
// Created by private on 05.12.23.
//

#ifndef CFORGESANDBOX_PLAYERSYSTEM_H
#define CFORGESANDBOX_PLAYERSYSTEM_H

#include <flecs/addons/cpp/world.hpp>
#include "crossforge/Graphics/VirtualCamera.h"
#include "crossforge/Input/Keyboard.h"
#include "crossforge/Input/Mouse.h"
#include "crossforge/Math/CForgeMath.h"
#include "Components.h"

namespace CForge {
    class PlayerSystem {
    public:
        static void addPlayerSystem(flecs::world &world) {
            world.system<PlayerComponent, PositionComponent>("PlayerSystem")
                    .iter([](flecs::iter it, PlayerComponent *p, PositionComponent *pc) {
                        for (int i: it) {
                            defaultCameraUpdate(p[i].pCamera, p[i].pKeyboard, p[i].pMouse, p[i].gameState, pc[i],
                                                4, 0.5f, 2.0f);
                        }
                    });
        }

    private:
        static void defaultCameraUpdate(VirtualCamera *pCamera, Keyboard *pKeyboard, Mouse *pMouse, GameState gameState,
                                        PositionComponent &pc,
                                        const float MovementSpeed = 0.4f, const float RotationSpeed = 1.0f,
                                        const float SpeedScale = 4.0f) {
            if (nullptr == pCamera) throw NullpointerExcept("pCamera");
            if (nullptr == pKeyboard) throw NullpointerExcept("pKeyboard");
            if (nullptr == pMouse) throw NullpointerExcept("pMouse");

            if (gameState == GAMEPLAY) {
                float S = 1.0f;
                if (pKeyboard->keyPressed(Keyboard::KEY_LEFT_SHIFT)) S = SpeedScale;

                auto forward = pCamera->dir();
                forward.y() = 0;
                auto right = pCamera->right();
                right.y() = 0;
                if (pKeyboard->keyPressed(Keyboard::KEY_W))
                    pc.translationDelta(forward * S * MovementSpeed);
                else if (pKeyboard->keyPressed(Keyboard::KEY_S))
                    pc.translationDelta(forward * S * -MovementSpeed);
                else if (pKeyboard->keyPressed(Keyboard::KEY_A))
                    pc.translationDelta(right * -S * MovementSpeed);
                else if (pKeyboard->keyPressed(Keyboard::KEY_D))
                    pc.translationDelta(right * S * MovementSpeed);
                else {
                    float ySpeed = pc.translationDelta().y() > 0 ? 0 : pc.translationDelta().y();
                    pc.translationDelta(Eigen::Vector3f(0, ySpeed, 0));
                }
                pCamera->position(pc.translation() + Eigen::Vector3f(0, PlayerComponent::HEIGHT, 0));

                const Eigen::Vector2f MouseDelta = pMouse->movement();
                const float pitchLimitUp = 87.0f; // Maximaler Pitch-Winkel nach oben (in Grad)
                const float pitchLimitDown = -87.0f; // Maximaler Pitch-Winkel nach unten (in Grad)
                const float pitchAmount = -0.1f * RotationSpeed * MouseDelta.y();

                const float currentPitch = pCamera->getPitch();

                // Überprüfen, ob die Mausbewegung ausreichend ist, um die Kamera zu drehen
                if (std::abs(MouseDelta.y()) > std::numeric_limits<float>::epsilon()) {
                    // Neuer Pitch-Winkel nach der Mausbewegung
                    const float newPitch = currentPitch + pitchAmount;

                    // Begrenzen des Pitch-Winkels innerhalb des zulässigen Bereichs
                    const float clampedPitch = std::clamp(newPitch, pitchLimitDown, pitchLimitUp);

                    // Änderung des Pitch-Winkels
                    const float pitchChange = clampedPitch - currentPitch;
                    pCamera->pitch(CForgeMath::degToRad(pitchChange));
                }

                if (std::abs(MouseDelta.x()) > std::numeric_limits<float>::epsilon()) {
                    pCamera->rotY(CForgeMath::degToRad(-0.1f * RotationSpeed * MouseDelta.x()));
                }
            }
            pMouse->movement(Eigen::Vector2f::Zero());
        }
    };
}
#endif //CFORGESANDBOX_PLAYERSYSTEM_H
