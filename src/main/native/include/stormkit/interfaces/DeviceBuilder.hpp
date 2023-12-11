#pragma once

/// A generic device builder. Takes in the type of the device as a parameter.
/// To implement this for a device, specialize the DeviceBuilder template, where T is the device type.
template <class T> class DeviceBuilder {
	/// Builds the device.
	/// @return The device.
	T Build();
};