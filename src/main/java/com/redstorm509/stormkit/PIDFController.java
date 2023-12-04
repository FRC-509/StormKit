package com.redstorm509.stormkit;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;

public class PIDFController extends PIDController {
	private double kF;

	public double getF() {
		return kF;
	}

	public void setF(double kF) {
		this.kF = kF;
	}

	public void setPIDF(double kP, double kI, double kD, double kF) {
		setPID(kP, kI, kD);
		setF(kF);
	}

	public PIDFController() {
		super(0.0d, 0.0d, 0.0d);
		setF(0.0d);
	}

	public PIDFController(double kP, double kI, double kD, double kF) {
		super(kP, kI, kD);
		setF(kF);
	}

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