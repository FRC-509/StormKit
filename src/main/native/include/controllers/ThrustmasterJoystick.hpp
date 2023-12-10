#pragma once
#include <frc/GenericHID.h>
#include <frc2/command/button/JoystickButton.h>

namespace stormkit {
    namespace controllers {
        class ThrustmasterJoystick : public frc::GenericHID {
        public:
            enum StickButton {
                Trigger = 1,
                Bottom = 2,
                Left = 3,
                Right = 4,
                LeftSideLeftTop = 5,
                LeftSideMiddleTop = 6,
                LeftSideRightTop = 7,
                LeftSideRightBottom = 8,
                LeftSideMiddleBottom = 9,
                LeftSideLeftBottom = 10,
                RightSideRightTop = 11,
                RightSideMiddleTop = 12,
                RightSideLeftTop = 13,
                RightSideLeftBottom = 14,
                RightSideMiddleBottom = 15,
                RightSideRightBottom = 16,
            };

            std::shared_ptr<frc2::JoystickButton> GetJoystickButton(StickButton button) {
                return std::make_shared<frc2::JoystickButton>(this, button);
            }

            bool IsPressed(StickButton button) {
                return GetRawButtonPressed(button);
            }

            void IsPressedBind(StickButton button, frc2::CommandPtr&& command) {
                GetJoystickButton(button)->ToggleOnTrue(std::move(command));
            }

            bool IsReleased(StickButton button) {
                return GetRawButtonReleased(button);
            }

            void IsReleasedBind(StickButton button, frc2::CommandPtr&& command) {
                GetJoystickButton(button)->ToggleOnFalse(std::move(command));
            }

            bool IsDown(StickButton button) {
                return GetRawButton(button);
            }

            void IsDownBind(StickButton button, frc2::CommandPtr&& command) {
                GetJoystickButton(button)->WhileTrue(std::move(command));
            }

            bool IsUp(StickButton button) {
                return !IsDown(button);
            }

            void IsUpBind(StickButton button, frc2::CommandPtr&& command) {
                GetJoystickButton(button)->WhileFalse(std::move(command));
            }
        };
    };
};
