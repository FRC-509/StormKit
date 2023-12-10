#pragma once
#include <frc/GenericHID.h>
#include <frc2/command/button/JoystickButton.h>

namespace stormkit {
    namespace controllers {
        class LogitechDualAction : public frc::GenericHID {
        public:
            enum LogiButton {
                A = 2,
                B = 3,
                X = 1,
                Y = 4,
                LBTrigger = 5,
                RBTrigger = 6,
                LTrigger = 7,
                RTrigger = 8,
                Back = 9,
                Start = 10,
                LStick = 11,
                RStick = 12,
            };

            enum LogiAxis {
                LeftStickX = 0,
                LeftStickY = 1,
                RightStickX = 2,
                RightStickY = 3,
            };

            std::shared_ptr<frc2::JoystickButton> GetControllerButton(LogiButton button) {
                return std::make_shared<frc2::JoystickButton>(this, button);
            }

            bool IsPressed(LogiButton button) {
                return GetRawButtonPressed(button);
            }

            void IsPressedBind(LogiButton button, frc2::CommandPtr&& command) {
                GetControllerButton(button)->ToggleOnTrue(std::move(command));
            }

            bool IsReleased(LogiButton button) {
                return GetRawButtonReleased(button);
            }

            void IsReleasedBind(LogiButton button, frc2::CommandPtr&& command) {
                GetControllerButton(button)->ToggleOnFalse(std::move(command));
            }

            bool IsDown(LogiButton button) {
                return GetRawButton(button);
            }

            void IsDownBind(LogiButton button, frc2::CommandPtr&& command) {
                GetControllerButton(button)->WhileTrue(std::move(command));
            }

            bool IsUp(LogiButton button) {
                return !IsDown(button);
            }

            void IsUpBind(LogiButton button, frc2::CommandPtr&& command) {
                GetControllerButton(button)->WhileFalse(std::move(command));
            }

            double GetLeftStickX() {
                return GetRawAxis(LogiAxis::LeftStickX);
            }

            double GetLeftStickY() {
                return GetRawAxis(LogiAxis::LeftStickY);
            }

            double GetRightStickX() {
                return GetRawAxis(LogiAxis::RightStickX);
            }

            double GetRightStickY() {
                return GetRawAxis(LogiAxis::RightStickY);
            }
        };
    };
};
