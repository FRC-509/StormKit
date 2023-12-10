#pragma once
#include <algorithm>
#include <limits>
#include <frc/Timer.h>

namespace stormkit {
    namespace math {
        class PositionTarget {
        private:
            double target = 0.0;
            double min_target_pos = -std::numeric_limits<double>::max();
            double max_target_pos = std::numeric_limits<double>::max();
            double max_rate = 1.0;

            double previous_time_stamp = 0.0;
        public:
            inline PositionTarget() {
                previous_time_stamp = frc::Timer::GetFPGATimestamp().value();
            }

            inline PositionTarget(double initial_target, double min_target_pos, double max_target_pos) {
                previous_time_stamp = frc::Timer::GetFPGATimestamp().value();
                this->min_target_pos = min_target_pos;
                this->max_target_pos = max_target_pos;
                this->target = std::clamp(initial_target, min_target_pos, max_target_pos);
            }

            inline PositionTarget(double initial_target, double min_target_pos, double max_target_pos, double max_rate) {
                previous_time_stamp = frc::Timer::GetFPGATimestamp().value();
                this->min_target_pos = min_target_pos;
                this->max_target_pos = max_target_pos;
                this->max_rate = max_rate;
                this->target = std::clamp(initial_target, min_target_pos, max_target_pos);
            }

            constexpr void SetMinTargetPosition(double min) {
                this->min_target_pos = min;
            }

            constexpr void SetMaxTargetPosition(double max) {
                this->max_target_pos = max;
            }

            constexpr void SetMaxRate(double max_rate) {
                this->max_rate = max_rate;
            }

            constexpr double SetTarget(double position) {
                target = std::clamp(position, min_target_pos, max_target_pos);
                return target;
            }

            inline double Update(double percent_of_maximum_rate) {
                percent_of_maximum_rate = std::clamp(percent_of_maximum_rate, -1.0, 1.0);

                double delta_time = frc::Timer::GetFPGATimestamp().value() - previous_time_stamp;
                target += max_rate * percent_of_maximum_rate * delta_time;
                previous_time_stamp = frc::Timer::GetFPGATimestamp().value();

                target = std::clamp(target, min_target_pos, max_target_pos);

                return target;
            }

            constexpr double GetTarget() const {
                return target;
            }
        };
    };
};