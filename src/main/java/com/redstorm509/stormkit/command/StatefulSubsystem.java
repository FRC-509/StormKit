package com.redstorm509.stormkit.command;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * An alternative way of organizing subsystem-specific code.
 * This generic class takes in an enum that represents the state of the subsytem, and provides interfaces for triggering code when states are entered and exited.
 */
public abstract class StatefulSubsystem<T extends Comparable<T>> extends SubsystemBase {
	protected T currentState;
	
    /**
     * Constructs an StatefulSubsystem with the given initial state.
     *
     * @param startingState The initial state of the subsystem. 
     */
	public StatefulSubsystem(T startingState) {
		this.currentState = startingState;
	}

	/**
     * Detects state changes and calls the enter/exit callbacks. This is called from the {@link StatefulSubsystem#periodic()} method.
     */
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

	/**
     * Returns the next state, implement this to facilitate state changes. This is called periodically in the {@link StatefulSubsystem#manageState()} method.
	 * @return The next state.
     */
	abstract public T getState();

	/**
     * Called once as the subsystem's state is changed. This is called when a new state is entered in the {@link StatefulSubsystem#manageState()} method.
	 * @param state The new state.
     */
	abstract public void onStateEnter(T state);

	/**
     * Called once as the subsystem's state is changed. This is called when a state is exited in the {@link StatefulSubsystem#manageState()} method.
	 * @param state The old state.
     */
	abstract public void onStateExit(T state);

	/**
     * This is called periodically in the {@link StatefulSubsystem#manageState()} method.
	 * @param state The current state.
     */
	abstract public void onStateUpdate(T state);
}