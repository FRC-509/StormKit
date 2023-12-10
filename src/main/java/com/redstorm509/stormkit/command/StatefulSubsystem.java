package com.redstorm509.stormkit.command;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class StatefulSubsystem<T extends Comparable<T>> extends Subsystem {
	T currentState;

	public StatefulSubsystem(T startingState) {
		this.currentState = startingState;
	}

	public void manageState() {
		if (DriverStation.isAutonomous()) {
			return;
		}

		T newState = getState();

		if (newState != currentState) {
			onStateExit(currentState);
			onStateEnter(newState);
			currentState = newState;
		}

		onStateUpdate(currentState);
	}

	@Override
	public void periodic() {
		manageState();
	}

	abstract public T getState();

	abstract public void onStateEnter(T state);

	abstract public void onStateExit(T state);

	abstract public void onStateUpdate(T state);
}