#pragma once
#include <algorithm>
#include <limits>
#include <frc/Timer.h>

namespace stormkit {
    namespace math {
        /// Represents a moving setpoint for a position PID with constraints and rate limiting.
        /// This class is perfect for implementing smoother approaches to a setpoint without having to tune a trapezoidal motion profile.
        class PositionTarget {
        private:
            double target = 0.0;
            double min_target_pos = -std::numeric_limits<double>::max();
            double max_target_pos = std::numeric_limits<double>::max();
            double max_rate = 1.0;
            double previous_time_stamp = 0.0;
        public:
            /// Constructs a PositionTarget with default parameters.
            inline PositionTarget() {
                previous_time_stamp = frc::Timer::GetFPGATimestamp().value();
            }

            /// Constructs a PositionTarget with an initial target and position constraints.
            /// @param initial_target The initial target position.
            /// @param min_target_pos The minimum allowed target position.
            /// @param max_target_pos The maximum allowed target position.
            inline PositionTarget(double initial_target, double min_target_pos, double max_target_pos) {
                previous_time_stamp = frc::Timer::GetFPGATimestamp().value();
                this->min_target_pos = min_target_pos;
                this->max_target_pos = max_target_pos;
                this->target = std::clamp(initial_target, min_target_pos, max_target_pos);
            }

            /// Constructs a PositionTarget with an initial target, position constraints, and maximum rate.
            /// @param initial_target The initial target position.
            /// @param min_target_pos The minimum allowed target position.
            /// @param max_target_pos The maximum allowed target position.
            /// @param max_rate The maximum rate of change for the target position.
            inline PositionTarget(double initial_target, double min_target_pos, double max_target_pos, double max_rate) {
                previous_time_stamp = frc::Timer::GetFPGATimestamp().value();
                this->min_target_pos = min_target_pos;
                this->max_target_pos = max_target_pos;
                this->max_rate = max_rate;
                this->target = std::clamp(initial_target, min_target_pos, max_target_pos);
            }

            /// Sets the minimum allowed target position.
            /// @param min The new minimum target position.
            constexpr void SetMinTargetPosition(double min) {
                this->min_target_pos = min;
            }

            /// Sets the maximum allowed target position.
            /// @param max The new maximum target position.
            constexpr void SetMaxTargetPosition(double max) {
                this->max_target_pos = max;
            }

            /// Sets the maximum rate of change for the target position.
            /// @param max_rate The new maximum rate.
            constexpr void SetMaxRate(double max_rate) {
                this->max_rate = max_rate;
            }

            /// Sets the target position, clamped within the specified constraints.
            /// @param position The new target position.
            /// @return The clamped target position.
            constexpr double SetTarget(double position) {
                target = std::clamp(position, min_target_pos, max_target_pos);
                return target;
            }

            /// Updates the target position based on the elapsed time and percent of maximum rate.
            /// @param percent_of_maximum_rate The percentage of the maximum rate of change [-1.0, 1.0].
            /// @return The updated target position.
            inline double Update(double percent_of_maximum_rate) {
                percent_of_maximum_rate = std::clamp(percent_of_maximum_rate, -1.0, 1.0);

                double delta_time = frc::Timer::GetFPGATimestamp().value() - previous_time_stamp;
                target += max_rate * percent_of_maximum_rate * delta_time;
                previous_time_stamp = frc::Timer::GetFPGATimestamp().value();

                target = std::clamp(target, min_target_pos, max_target_pos);

                return target;
            }

            /// Gets the current target position.
            /// @return The current target position.
            constexpr double GetTarget() const {
                return target;
            }
        };
    };
};