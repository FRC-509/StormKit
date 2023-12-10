package com.redstorm509.stormkit.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

public class PositionTarget {
	double target = 0.0;
	double minTargetPos = -Double.MAX_VALUE;
	double maxTargetPos = +Double.MAX_VALUE;
	double maxRate = 1.0;

	double previousTimeStamp = 0.0;

	public PositionTarget() {
		previousTimeStamp = Timer.getFPGATimestamp();
	}

	public PositionTarget(double initialTarget, double minTargetPos, double maxTargetPos) {
		previousTimeStamp = Timer.getFPGATimestamp();
		this.minTargetPos = minTargetPos;
		this.maxTargetPos = maxTargetPos;
		setTarget(initialTarget);
	}

	public PositionTarget(double initialTarget, double minTargetPos, double maxTargetPos, double maxRate) {
		previousTimeStamp = Timer.getFPGATimestamp();
		this.minTargetPos = minTargetPos;
		this.maxTargetPos = maxTargetPos;
		this.maxRate = maxRate;
		setTarget(initialTarget);
	}

	public void setMinTargetPosition(double min) {
		this.minTargetPos = min;
	}

	public void setMaxTargetPosition(double max) {
		this.maxTargetPos = max;
	}

	public void setMaxRate(double maxRate) {
		this.maxRate = maxRate;
	}

	public double setTarget(double position) {
		target = MathUtil.clamp(position, minTargetPos, maxTargetPos);
		return target;
	}

	public double update(double percentOfMaximumRate) {
		percentOfMaximumRate = MathUtil.clamp(percentOfMaximumRate, -1.0d, 1.0d);

		double deltaTime = Timer.getFPGATimestamp() - previousTimeStamp;
		target += maxRate * percentOfMaximumRate * deltaTime;
		previousTimeStamp = Timer.getFPGATimestamp();

		target = MathUtil.clamp(target, minTargetPos, maxTargetPos);

		return target;
	}

	public double getTarget() {
		return target;
	}
}