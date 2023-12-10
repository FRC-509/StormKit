package com.redstorm509.stormkit.math;

public class Interpolator {
	double scale;
	double setPoint;
	double trailingPoint;
	double position;

	public Interpolator(double scale) {
		this.scale = scale;
		this.setPoint = 0.0d;
		this.trailingPoint = 0.0d;
		this.position = 0.0d;
	}

	public void setPoint(double value) {
		setPoint = value;
	}

	public double getPosition() {
		return position;
	}

	public double update(double deltaTimeSeconds) {
		position += scale * (trailingPoint - position) * deltaTimeSeconds;
		boolean withinDeadband = Math.abs(trailingPoint - position) <= 0.1 * scale;
		if (withinDeadband) {
			trailingPoint = setPoint;
		}

		return position;
	}
}