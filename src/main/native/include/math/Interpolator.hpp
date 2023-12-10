#pragma once
#include <cmath>

namespace stormkit {
	namespace math {
		class Interpolator {
			double scale;
			double set_point;
			double trailing_point;
			double position;

			constexpr Interpolator(double scale) : scale(scale), set_point(0), trailing_point(0), position(0) {
			}

			constexpr void SetPoint(double value) {
				set_point = value;
			}

			constexpr double GetPosition() const {
				return position;
			}

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