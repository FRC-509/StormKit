package com.redstorm509.stormkit;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Implements a PIDF control loop. */
public class PIDFController extends PIDController {
	private double kF = 0.0d;

	/**
	 * Allocates a PIDFController with the given constants for kp, ki, kd, and kf and a default period of
	 * 0.02 seconds.
	 *
	 * @param kp The proportional coefficient.
	 * @param ki The integral coefficient.
	 * @param kd The derivative coefficient.
	 * @param kf The feedforward coefficient.
	 */
	public PIDFController(double kp, double ki, double kd, double kf) {
		super(kp, ki, kd);
		setF(kf);
	}

	/**
	 * Allocates a PIDFController with the given constants for kp, ki, kd, and kf.
	 *
	 * @param kp The proportional coefficient.
	 * @param ki The integral coefficient.
	 * @param kd The derivative coefficient.
	 * @param kf The feedforward coefficient.
	 * @param period The period between controller updates in seconds. Must be non-zero and positive.
	 */
	public PIDFController(double kp, double ki, double kd, double kf, double period) {
		super(kp, ki, kd, period);
		setF(kf);
	}

	/**
	 * Get the Feedforward coefficient.
	 *
	 * @return feedforward coefficient
	 */
	public double getF() {
		return kF;
	}

	/**
	 * Sets the Feedforward coefficient of the PIDF controller gain.
	 *
	 * @param kf feedforward coefficient
	 */
	public void setF(double kf) {
		this.kF = kf;
	}

	/**
	 * Sets the PIDF Controller gain parameters.
	 *
	 * <p>Set the proportional, integral, differential, and feedforward coefficients.
	 *
	 * @param kp The proportional coefficient.
	 * @param ki The integral coefficient.
	 * @param kd The derivative coefficient.
	 * @param kf The feedforward coefficient.
	 */
	public void setPIDF(double kp, double ki, double kd, double kf) {
		setPID(kp, ki, kd);
		setF(kf);
	}

	/**
	 * Returns the next output of the PID controller.
	 *
	 * @param measurement The current measurement of the process variable.
	 * @return The next controller output.
	 */
	@Override
	public double calculate(double measurement) {
		return super.calculate(measurement) + measurement * kF;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("f", this::getF, this::setF);
	}
}