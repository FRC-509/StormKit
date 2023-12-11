package com.redstorm509.stormkit.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Handle input from Logitech Dual Action controllers connected to the Driver Station.
*/
public class LogitechDualAction extends GenericHID {
	public enum LogiButton {
		A(2),
		B(3),
		X(1),
		Y(4),
		LBTrigger(5),
		RBTrigger(6),
		LTrigger(7),
		RTrigger(8),
		Back(9),
		Start(10),
		LStick(11),
		RStick(12);

		public final int id;

		LogiButton(int id) {
			this.id = id;
		}

		public int id() {
			return id;
		}
	}

	public enum LogiAxis {
		LeftStickX(0),
		LeftStickY(1),
		RightStickX(2),
		RightStickY(3);

		public final int id;

		LogiAxis(int id) {
			this.id = id;
		}

		public int id() {
			return id;
		}
	}

	/**
	 * Constructs an instance of a Logitech Dual Action controller.
	 *
	 * @param port The port index on the Driver Station that the device is plugged into.
	 */
	public LogitechDualAction(int port) {
		super(port);
	}

	/**
	 * Returns a joystick button for triggering commands.
	 *
	 * @param button The physical button on the controller.
	 * @return A JoystickButton object attached to the given controller button.
	 */
	public JoystickButton getControllerButton(LogiButton button) {
		return new JoystickButton(this, button.id());
	}

	/**
	 * Whether the button was pressed since the last check.
	 * <p>This method returns true if the button went from not pressed to held down since the last time this method was called. This is useful if you only want to call a function once when you press the button.
	 *
	 * @param button The physical button on the controller.
	 * @return Whether the button was pressed since the last check.
	 */
	public boolean isPressed(LogiButton button) {
		return super.getRawButtonPressed(button.id());
	}

	/**
	 * Binds a command to when a button is pressed.
	 * <p>The command is ran if the button went from not pressed to held down. This is useful if you only want to execute a command once when you press the button.
	 *
	 * @param button The physical button on the controller.
	 * @param command The command to execute once the button is pressed.
	 */
	public void isPressedBind(LogiButton button, Command command) {
		getControllerButton(button).toggleOnTrue(command);
	}

	/**
	 * Whether the button was released since the last check.
	 * <p>This method returns true if the button went from held down to not pressed since the last time this method was called. This is useful if you only want to call a function once when you release the button.
	 * 
	 * @param button The physical button on the controller.
	 * @return Whether the button was released since the last check.
	 */
	public boolean isReleased(LogiButton button) {
		return super.getRawButtonReleased(button.id());
	}

	/**
	 * Binds a command to when a button is released.
	 * <p>The command is ran if the button went from held down to not pressed. This is useful if you only want to execute a command once when you release the button.
	 *
	 * @param button The physical button on the controller.
	 * @param command The command to execute once the button is released.
	 */
	public void isReleasedBind(LogiButton button, Command command) {
		getControllerButton(button).toggleOnFalse(command);
	}

	/**
	 * Get the button value.
	 * <p>This method returns true if the button is being held down at the time that this method is being called.
	 *
	 * @param button The physical button on the controller.
	 * @return The state of the button.
	 */
	public boolean isDown(LogiButton button) {
		return super.getRawButton(button.id());
	}

	/**
	 * Binds a command to while a button is held down. The given command is started when the button is initially pressed and is cancelled when the button is released.
	 * <p>Doesn't re-start the command if it ends while the button is still pressed. If the command should restart, see {@link edu.wpi.first.wpilibj2.command.RepeatCommand}.
	 *
	 * @param button The physical button on the controller.
	 * @param command The command to execute while the button is pressed.
	 */	
	public void isDownBind(LogiButton button, Command command) {
		getControllerButton(button).whileTrue(command);
	}

	/**
	 * This method returns false if the button is being held down at the time that this method is being called.
	 *
	 * @param button The physical button on the controller.
	 * @return Whether the button is up or not.
	 */
	public boolean isUp(LogiButton button) {
		return !isDown(button);
	}

	/**
	 * Binds a command to while a button is not pressed. The given command is started when the button is initially released and is cancelled when the button is pressed.
	 * <p>Doesn't re-start the command if it ends while the button still has not been pressed. If the command should restart, see {@link edu.wpi.first.wpilibj2.command.RepeatCommand}.
	 *
	 * @param button The physical button on the controller.
	 * @param command The command to execute while the button is not pressed.
	 */	
	public void isUpBind(LogiButton button, Command command) {
		getControllerButton(button).whileFalse(command);
	}

	/**
	 * Get the value of the x-axis on the left stick.
	 *
	 * @return The value of the left stick's x-axis.
	 */
	public double getLeftStickX() {
		return super.getRawAxis(LogiAxis.LeftStickX.id());
	}

	/**
	 * Get the value of the y-axis on the left stick.
	 *
	 * @return The value of the left stick's y-axis.
	 */
	public double getLeftStickY() {
		return super.getRawAxis(LogiAxis.LeftStickY.id());
	}

	/**
	 * Get the value of the x-axis on the right stick.
	 *
	 * @return The value of the right stick's x-axis.
	 */
	public double getRightStickX() {
		return super.getRawAxis(LogiAxis.RightStickX.id());
	}

	/**
	 * Get the value of the y-axis on the right stick.
	 *
	 * @return The value of the right stick's y-axis.
	 */
	public double getRightStickY() {
		return super.getRawAxis(LogiAxis.RightStickY.id());
	}
}