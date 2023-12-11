#pragma once
#include <frc/GenericHID.h>
#include <frc2/command/button/JoystickButton.h>

namespace stormkit {
    namespace controllers {
        /// Handle input from Logitech Dual Action controllers connected to the Driver Station.
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

            /// Returns a joystick button for triggering commands.
            /// @param button The physical button on the controller.
            /// @return A JoystickButton object attached to the given controller button.
            std::shared_ptr<frc2::JoystickButton> GetControllerButton(LogiButton button) {
                return std::make_shared<frc2::JoystickButton>(this, button);
            }

            /// Whether the button was pressed since the last check.
            /// <p>This method returns true if the button went from not pressed to held down since the last time this method was called. This is useful if you only want to call a function once when you press the button.
            /// @param button The physical button on the controller.
            /// @return Whether the button was pressed since the last check.
            bool IsPressed(LogiButton button) {
                return GetRawButtonPressed(button);
            }

            /// Binds a command to when a button is pressed.
            /// <p>The command is ran if the button went from not pressed to held down. This is useful if you only want to execute a command once when you press the button.
            /// @param button The physical button on the controller.
            /// @param command The command to execute once the button is pressed.
            void IsPressedBind(LogiButton button, frc2::CommandPtr&& command) {
                GetControllerButton(button)->ToggleOnTrue(std::move(command));
            }

            /// Whether the button was released since the last check.
            /// <p>This method returns true if the button went from held down to not pressed since the last time this method was called. This is useful if you only want to call a function once when you release the button.
            /// @param button The physical button on the controller.
            /// @return Whether the button was released since the last check.
            bool IsReleased(LogiButton button) {
                return GetRawButtonReleased(button);
            }

            /// Binds a command to when a button is released.
            /// <p>The command is ran if the button went from held down to not pressed. This is useful if you only want to execute a command once when you release the button.
            /// @param button The physical button on the controller.
            /// @param command The command to execute once the button is released.
            void IsReleasedBind(LogiButton button, frc2::CommandPtr&& command) {
                GetControllerButton(button)->ToggleOnFalse(std::move(command));
            }

            /// Get the button value.
            /// <p>This method returns true if the button is being held down at the time that this method is being called.
            /// @param button The physical button on the controller.
            /// @return The state of the button.
            bool IsDown(LogiButton button) {
                return GetRawButton(button);
            }

            /// Binds a command to while a button is held down. The given command is started when the button is initially pressed and is cancelled when the button is released.
            /// <p>Doesn't re-start the command if it ends while the button is still pressed. If the command should restart, see {@link edu.wpi.first.wpilibj2.command.RepeatCommand}.
            /// @param button The physical button on the controller.
            /// @param command The command to execute while the button is pressed.
            void IsDownBind(LogiButton button, frc2::CommandPtr&& command) {
                GetControllerButton(button)->WhileTrue(std::move(command));
            }

            /// This method returns false if the button is being held down at the time that this method is being called.
            /// @param button The physical button on the controller.
            /// @return Whether the button is up or not.
            bool IsUp(LogiButton button) {
                return !IsDown(button);
            }

            /// Binds a command to while a button is not pressed. The given command is started when the button is initially released and is cancelled when the button is pressed.
            /// <p>Doesn't re-start the command if it ends while the button still has not been pressed. If the command should restart, see {@link edu.wpi.first.wpilibj2.command.RepeatCommand}.
            /// @param button The physical button on the controller.
            /// @param command The command to execute while the button is not pressed.
            void IsUpBind(LogiButton button, frc2::CommandPtr&& command) {
                GetControllerButton(button)->WhileFalse(std::move(command));
            }

            /// Get the value of the x-axis on the left stick.
            /// @return The value of the left stick's x-axis.
            double GetLeftStickX() {
                return GetRawAxis(LogiAxis::LeftStickX);
            }

            /// Get the value of the y-axis on the left stick.
            /// @return The value of the left stick's y-axis.
            double GetLeftStickY() {
                return GetRawAxis(LogiAxis::LeftStickY);
            }

            /// Get the value of the x-axis on the right stick.
            /// @return The value of the right stick's x-axis.
            double GetRightStickX() {
                return GetRawAxis(LogiAxis::RightStickX);
            }

            /// Get the value of the y-axis on the right stick.
            /// @return The value of the right stick's y-axis.
            double GetRightStickY() {
                return GetRawAxis(LogiAxis::RightStickY);
            }
        };
    };
};
