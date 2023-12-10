/*
 * An example for implementing the device builder interface on DoubleSolenoid.
 */

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import com.redstorm509.stormkit.interfaces.IDeviceBuilder;

public class DeviceBuilder {
	public static class DoubleSolenoidBuilder implements IDeviceBuilder<DoubleSolenoid> {
		final int id;
		final int forwardChannel;
		final int reverseChannel;
		final PneumaticsModuleType type;

		public DoubleSolenoidBuilder(int id, int forwardChannel, int reverseChannel, PneumaticsModuleType type) {
			this.id = id;
			this.forwardChannel = forwardChannel;
			this.reverseChannel = reverseChannel;
			this.type = type;
		}

		@Override
		public DoubleSolenoid build() {
			return new DoubleSolenoid(id, type, forwardChannel, reverseChannel);
		}
	}
}