/*
 * An example for implementing the device builder interface on DoubleSolenoid.
 */

#include <frc/DoubleSolenoid.h>
#include <stormkit/interfaces/DeviceBuilder.hpp>

template<> class DeviceBuilder<frc::DoubleSolenoid> {
private:
    int id;
    int forward_channel;
    int reverse_channel;
    frc::PneumaticsModuleType type;
public:
	DeviceBuilder(int id, int forward_channel, int reverse_channel, frc::PneumaticsModuleType type) {
		this->id = id;
        this->forward_channel = forward_channel;
        this->reverse_channel = reverse_channel;
        this->type = type;
	}
	frc::DoubleSolenoid Build() {
		return frc::DoubleSolenoid{id, type, forward_channel, reverse_channel};
	}
};