#pragma once
#include <vector>
#include <algorithm>
#include <Eigen/Core>

namespace stormkit {
    namespace math {
        template <std::size_t MAX_ITERATIONS = 100> class IKSolver {
        public:
            struct SegmentParams {
                double pivot_min_extent_radians;
                double pivot_max_extent_radians;
                double pivot_radians;
                double length_meters;
                inline void apply_delta(const double delta) {
                    this->pivot_radians = std::clamp(this->pivot_radians + delta, this->pivot_min_extent_radians, this->pivot_max_extent_radians);
                }
            };
        private:
            std::vector<SegmentParams> segments;
        public:
            /// @brief Constructs an IKSolver with the given segment information.
            /// @param segment_params 
            inline IKSolver(const std::vector<SegmentParams>& segment_params) : segments(segment_params) {
            }

            /// @brief Constructs an IKSolver with the given segment information. Note: This function takes ownership of the vector passed in!
            /// @param segment_params
            inline IKSolver(std::vector<SegmentParams>&& segment_params) : segments(std::move(segment_params)) {
            }

            /// @brief Updates the segment information. Run this with sensor information before doing any calculations.
            /// @param segment_params An ArrayList of 2-component vectors, where each vector's x-component corresponds to a joint's pivot angle in radians, and its y-component corresponds to a joint's arm length in meters. 
            inline void update_segments(const std::vector<SegmentParams>& segment_params) {
                this->segments = segment_params;
            }

            /// @brief Updates the segment information. Run this with sensor information before doing any calculations. Note: This function takes ownership of the vector passed in!
            /// @param segment_params An ArrayList of 2-component vectors, where each vector's x-component corresponds to a joint's pivot angle in radians, and its y-component corresponds to a joint's arm length in meters.  
            inline void update_segments(std::vector<SegmentParams>&& segment_params) {
                this->segments = std::move(segment_params);
            }

            /// @brief Gets the current segment data. Run this after performing calculations.
            /// @return An ArrayList of 2-component vectors, where each vector's x-component corresponds to a joint's pivot angle in radians, and its y-component corresponds to a joint's arm length in meters. 
            constexpr std::vector<SegmentParams> get_segments() const {
                return this->segments;
            }

            /// @brief Performs forward kinematics on the arm segments, and returns an ArrayList of joint positions in meters.
            /// @return An ArrayList of joint position vectors in meters.
            inline std::vector<Eigen::Vector2d> forward_kinematics() {
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
            inline void inverse_kinematics(Eigen::Vector2d desiredPos) {
                for (std::size_t n = 0; n < MAX_ITERATIONS; n++) {
                    for (std::size_t i = 0; i < this->segments.size(); i++) {
                        std::vector<Eigen::Vector2d> positions = this->forwardKinematics();

                        // position of current segment
                        const Eigen::Vector2d& currentSegmentPos = positions[i];
                        // position of end effector
                        const Eigen::Vector2d& endEffectorPos = positions[positions.size() - 1];

                        if ((desiredPos - endEffectorPos).norm() < 0.01) {
                            return;
                        }

                        const Eigen::Vector2d currentSegmentToEndEffector = endEffectorPos - currentSegmentPos;
                        const Eigen::Vector2d currentSegmentToDesiredPos = desiredPos - currentSegmentPos;

                        // compute the angle of the triangle created between
                        // the current segment point, the last segment point,
                        // and the desired end position
                        double a = currentSegmentToEndEffector.norm();
                        double b = currentSegmentToDesiredPos.norm();
                        double dot = currentSegmentToEndEffector.dot(currentSegmentToDesiredPos);
                        double delta = std::acos(dot / (a * b));

                        // calculate whether delta calculates for the necessary positive
                        // or negative offset to the current segment angle using its normal
                        double direction = -currentSegmentToEndEffector.y() * currentSegmentToDesiredPos.x() + currentSegmentToEndEffector.x() * currentSegmentToDesiredPos.y();

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