package com.redstorm509.stormkit.math;

import com.redstorm509.stormkit.command.TimeStamp;

public class Interpolator {
	double scale;
	double setPoint;
	double trailingPoint;
	double position;

	TimeStamp timeStamp;

	public Interpolator(TimeStamp timeStamp, double scale) {
		this.scale = scale;
		this.timeStamp = timeStamp;
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

	public double update() {
		position += scale * (trailingPoint - position) * timeStamp.deltaTime();
		boolean withinDeadband = Math.abs(trailingPoint - position) <= 0.1 * scale;
		if (withinDeadband) {
			trailingPoint = setPoint;
		}

		return position;
	}
}