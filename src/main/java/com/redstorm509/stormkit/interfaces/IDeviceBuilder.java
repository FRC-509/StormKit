package com.redstorm509.stormkit.interfaces;

/**
 * A generic device builder. Takes in the type of the device as a parameter.
 * To implement this for a device, create a new class that implements IDeviceBuilder<T>, where T is the device.
 */
public interface IDeviceBuilder<T> {
	/**
	 * Builds the device.
	 *
	 * @return The device.
	 */
	public T build();
}