#pragma once
#include <vector>
#include <algorithm>
#include <Eigen/Core>

namespace stormkit {
    namespace math {
        /// A simple 2-dimensional inverse kinematics solver.
        /// Reference code: https://github.com/FRC-509/inverse-kinematics/blob/master/src/kinematics.rs
        template <std::size_t MAX_ITERATIONS = 100> class IKSolver {
        public:
            struct SegmentParams {
                /// The minimum pivot angle permitted for the segment in radians.
                double pivot_min_extent_radians;
                /// The maximum pivot angle permitted for the segment in radians.
                double pivot_max_extent_radians;
                /// The initial angle of the segment in radians.
                double pivot_radians;
                /// The length of the segment in meters.
                double length_meters;
                constexpr void apply_delta(const double delta) {
                    this->pivot_radians = std::clamp(this->pivot_radians + delta, this->pivot_min_extent_radians, this->pivot_max_extent_radians);
                }
            };
        private:
            std::vector<SegmentParams> segments;
        public:
            /// @brief Constructs an IKSolver with the given segment information.
            /// @param segment_params 
            constexpr IKSolver(const std::vector<SegmentParams>& segment_params) : segments(segment_params) {
            }

            /// @brief Constructs an IKSolver with the given segment information. Note: This function takes ownership of the vector passed in!
            /// @param segment_params
            constexpr IKSolver(std::vector<SegmentParams>&& segment_params) : segments(std::move(segment_params)) {
            }

            /// @brief Updates the segment information. Run this with sensor information before doing any calculations.
            /// @param segment_params An ArrayList of 2-component vectors, where each vector's x-component corresponds to a joint's pivot angle in radians, and its y-component corresponds to a joint's arm length in meters. 
            constexpr void UpdateSegments(const std::vector<SegmentParams>& segment_params) {
                this->segments = segment_params;
            }

            /// @brief Updates the segment information. Run this with sensor information before doing any calculations. Note: This function takes ownership of the vector passed in!
            /// @param segment_params An ArrayList of 2-component vectors, where each vector's x-component corresponds to a joint's pivot angle in radians, and its y-component corresponds to a joint's arm length in meters.  
            constexpr void UpdateSegments(std::vector<SegmentParams>&& segment_params) {
                this->segments = std::move(segment_params);
            }

            /// @brief Gets the current segment data. Run this after performing calculations.
            /// @return An ArrayList of 2-component vectors, where each vector's x-component corresponds to a joint's pivot angle in radians, and its y-component corresponds to a joint's arm length in meters. 
            constexpr std::vector<SegmentParams> GetSegments() const {
                return this->segments;
            }

            /// @brief Performs forward kinematics on the arm segments, and returns an ArrayList of joint positions in meters.
            /// @return An ArrayList of joint position vectors in meters.
            inline std::vector<Eigen::Vector2d> ForwardKinematics() {
                std::vector<Eigen::Vector2d> positions{};
                positions.reserve(this->segments.size());
                double theta = 0.0;
                double x = 0.0;
                double y = 0;

                for (const auto& segment : this->segments) {
                    double pivot_radians = segment.pivot_radians;
                    double length_meters = segment.length_meters;
                    theta += pivot_radians;
                    x += std::cos(theta) * length_meters;
                    y += std::sin(theta) * length_meters;

                    positions.emplace_back(x, y);
                }

                return std::move(positions);
            }

            /// @brief Performs inverse kinematics on the arm segments given a desired end-effector position.
            /// @param desiredPos A desired end effector position in meters.
            inline void InverseKinematics(Eigen::Vector2d desiredPos) {
                for (std::size_t n = 0; n < MAX_ITERATIONS; n++) {
                    for (std::size_t i = 0; i < this->segments.size(); i++) {
                        std::vector<Eigen::Vector2d> positions = this->forwardKinematics();

                        // position of current segment
                        const Eigen::Vector2d& current_segment_pos = positions[i];
                        // position of end effector
                        const Eigen::Vector2d& end_effector_pos = positions[positions.size() - 1];

                        if ((desiredPos - end_effector_pos).norm() < 0.01) {
                            return;
                        }

                        const Eigen::Vector2d current_segment_to_end_effector = end_effector_pos - current_segment_pos;
                        const Eigen::Vector2d current_segment_to_desired_pos = desiredPos - current_segment_pos;

                        // compute the angle of the triangle created between
                        // the current segment point, the last segment point,
                        // and the desired end position
                        double a = current_segment_to_end_effector.norm();
                        double b = current_segment_to_desired_pos.norm();
                        double dot = current_segment_to_end_effector.dot(current_segment_to_desired_pos);
                        double delta = std::acos(dot / (a * b));

                        // calculate whether delta calculates for the necessary positive
                        // or negative offset to the current segment angle using its normal
                        double direction = -current_segment_to_end_effector.y() * current_segment_to_desired_pos.x() + current_segment_to_end_effector.x() * current_segment_to_desired_pos.y();

                        if (std::abs(delta) < 0.0001) {
                            if (direction < 0) {
                                this->segments[i].apply_delta(-delta);
                            }
                            else {
                                this->segments[i].apply_delta(delta);
                            }
                        }
                    }
                }
            }
        };
    };
};