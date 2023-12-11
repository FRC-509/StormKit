#pragma once
#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>

namespace stormkit {
    namespace controllers {
        /// Handle input from Thrustmaster T.16000M joysticks connected to the Driver Station.
        class ThrustmasterJoystick : public frc::Joystick {
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

            /// Returns a joystick button for triggering commands.
            /// @param button The physical button on the controller.
            /// @return A JoystickButton object attached to the given controller button.
            std::shared_ptr<frc2::JoystickButton> GetJoystickButton(StickButton button) {
                return std::make_shared<frc2::JoystickButton>(this, button);
            }

            /// Whether the button was pressed since the last check.
            /// <p>This method returns true if the button went from not pressed to held down since the last time this method was called. This is useful if you only want to call a function once when you press the button.
            /// @param button The physical button on the controller.
            /// @return Whether the button was pressed since the last check.
            bool IsPressed(StickButton button) {
                return GetRawButtonPressed(button);
            }

            /// Binds a command to when a button is pressed.
            /// <p>The command is ran if the button went from not pressed to held down. This is useful if you only want to execute a command once when you press the button.
            /// @param button The physical button on the controller.
            /// @param command The command to execute once the button is pressed.
            void IsPressedBind(StickButton button, frc2::CommandPtr&& command) {
                GetJoystickButton(button)->ToggleOnTrue(std::move(command));
            }

            /// Whether the button was released since the last check.
            /// <p>This method returns true if the button went from held down to not pressed since the last time this method was called. This is useful if you only want to call a function once when you release the button.
            /// @param button The physical button on the controller.
            /// @return Whether the button was released since the last check.
            bool IsReleased(StickButton button) {
                return GetRawButtonReleased(button);
            }

            /// Binds a command to when a button is released.
            /// <p>The command is ran if the button went from held down to not pressed. This is useful if you only want to execute a command once when you release the button.
            /// @param button The physical button on the controller.
            /// @param command The command to execute once the button is released.
            void IsReleasedBind(StickButton button, frc2::CommandPtr&& command) {
                GetJoystickButton(button)->ToggleOnFalse(std::move(command));
            }

            /// Get the button value.
            /// <p>This method returns true if the button is being held down at the time that this method is being called.
            /// @param button The physical button on the controller.
            /// @return The state of the button.
            bool IsDown(StickButton button) {
                return GetRawButton(button);
            }

            /// Binds a command to while a button is held down. The given command is started when the button is initially pressed and is cancelled when the button is released.
            /// <p>Doesn't re-start the command if it ends while the button is still pressed. If the command should restart, see {@link edu.wpi.first.wpilibj2.command.RepeatCommand}.
            /// @param button The physical button on the controller.
            /// @param command The command to execute while the button is pressed.
            void IsDownBind(StickButton button, frc2::CommandPtr&& command) {
                GetJoystickButton(button)->WhileTrue(std::move(command));
            }

            /// This method returns false if the button is being held down at the time that this method is being called.
            /// @param button The physical button on the controller.
            /// @return Whether the button is up or not.
            bool IsUp(StickButton button) {
                return !IsDown(button);
            }

            /// Binds a command to while a button is not pressed. The given command is started when the button is initially released and is cancelled when the button is pressed.
            /// <p>Doesn't re-start the command if it ends while the button still has not been pressed. If the command should restart, see {@link edu.wpi.first.wpilibj2.command.RepeatCommand}.
            /// @param button The physical button on the controller.
            /// @param command The command to execute while the button is not pressed.
            void IsUpBind(StickButton button, frc2::CommandPtr&& command) {
                GetJoystickButton(button)->WhileFalse(std::move(command));
            }
        };
    };
};
