package com.redstorm509.stormkit.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

/**
 * Represents a moving setpoint for a position PID with constraints and rate limiting.
 * This class is perfect for implementing smoother approaches to a setpoint without having to tune a trapezoidal motion profile.
 */
public class PositionTarget {
	private double target = 0.0;
	private double minTargetPos = -Double.MAX_VALUE;
	private double maxTargetPos = +Double.MAX_VALUE;
	private double maxRate = 1.0;
	private double previousTimeStamp = 0.0;

    /**
     * Constructs a PositionTarget with default parameters.
     */
	public PositionTarget() {
		previousTimeStamp = Timer.getFPGATimestamp();
	}

    /**
     * Constructs a PositionTarget with an initial target and position constraints.
     * @param initialTarget The initial target position.
     * @param minTargetPos The minimum allowed target position.
     * @param maxTargetPos The maximum allowed target position.
     */
	public PositionTarget(double initialTarget, double minTargetPos, double maxTargetPos) {
		previousTimeStamp = Timer.getFPGATimestamp();
		this.minTargetPos = minTargetPos;
		this.maxTargetPos = maxTargetPos;
		setTarget(initialTarget);
	}

    /**
     * Constructs a PositionTarget with an initial target, position constraints, and maximum rate.
     * @param initialTarget The initial target position.
     * @param minTargetPos The minimum allowed target position.
     * @param maxTargetPos The maximum allowed target position.
     * @param maxRate The maximum rate of change for the target position.
     */
	public PositionTarget(double initialTarget, double minTargetPos, double maxTargetPos, double maxRate) {
		previousTimeStamp = Timer.getFPGATimestamp();
		this.minTargetPos = minTargetPos;
		this.maxTargetPos = maxTargetPos;
		this.maxRate = maxRate;
		setTarget(initialTarget);
	}

    /**
     * Sets the minimum allowed target position.
     * @param min The new minimum target position.
     */
    public void setMinTargetPosition(double min) {
        this.minTargetPos = min;
    }

    /**
     * Sets the maximum allowed target position.
     * @param max The new maximum target position.
     */
    public void setMaxTargetPosition(double max) {
        this.maxTargetPos = max;
    }

    /**
     * Sets the maximum rate of change for the target position.
     * @param maxRate The new maximum rate.
     */
    public void setMaxRate(double maxRate) {
        this.maxRate = maxRate;
    }

    /**
     * Sets the target position, clamped within the specified constraints.
     * @param position The new target position.
     * @return The clamped target position.
     */
	public double setTarget(double position) {
		target = MathUtil.clamp(position, minTargetPos, maxTargetPos);
		return target;
	}

    /**
     * Updates the target position based on the elapsed time and percent of maximum rate.
     * @param percentOfMaximumRate The percentage of the maximum rate of change [-1.0, 1.0].
     * @return The updated target position.
     */
	public double update(double percentOfMaximumRate) {
		percentOfMaximumRate = MathUtil.clamp(percentOfMaximumRate, -1.0d, 1.0d);

		double deltaTime = Timer.getFPGATimestamp() - previousTimeStamp;
		target += maxRate * percentOfMaximumRate * deltaTime;
		previousTimeStamp = Timer.getFPGATimestamp();

		target = MathUtil.clamp(target, minTargetPos, maxTargetPos);

		return target;
	}

    /**
     * Gets the current target position.
     * @return The current target position.
     */
	public double getTarget() {
		return target;
	}
}