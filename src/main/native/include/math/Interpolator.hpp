#pragma once
#include <cmath>

namespace stormkit {
	namespace math {
		/// Represents a simple linear interpolator.
		class Interpolator {
		private:
			double scale;
			double set_point;
			double trailing_point;
			double position;

			/// Constructs an Interpolator with the specified scale.
			/// @param scale The scaling factor for the interpolation.
			constexpr Interpolator(double scale) : scale(scale), set_point(0), trailing_point(0), position(0) {
			}

			/// Sets the target set point for the interpolation.
			/// @param value The new set point value.
			constexpr void SetPoint(double value) {
				set_point = value;
			}

			/// Gets the current position obtained through interpolation.
			/// @return The current position.
			constexpr double GetPosition() const {
				return position;
			}

			/// Updates the interpolation based on the elapsed time.
			/// @param delta_time_seconds The time elapsed in seconds since the last update.
			/// @return The updated position after interpolation.
			constexpr double Update(double delta_time_seconds) {
				position += scale * (trailing_point - position) * delta_time_seconds;
				bool within_deadband = std::abs(trailing_point - position) <= 0.1 * scale;
				if (within_deadband) {
					trailing_point = set_point;
				}

				return position;
			}
		};
	};
};