#pragma once
#include <frc/controller/PIDController.h>
#include <wpi/sendable/SendableBuilder.h>

namespace stormkit {
    /// Implements a PIDF control loop.
    class PIDFController : public frc::PIDController {
    private:
        double kF = 0.0f;
    public:

        /// Allocates a PIDFController with the given constants for kp, ki, kd, and kf and a default period of 0.02 seconds.
        /// @param kp The proportional coefficient.
        /// @param ki The integral coefficient.
        /// @param kd The derivative coefficient.
        /// @param kf The feedforward coefficient.
        inline PIDFController(double kp, double ki, double kd, double kf) : PIDController(kp, ki, kd) {
            SetF(kf);
        }

        /// Allocates a PIDFController with the given constants for kp, ki, kd, and kf.
        /// @param kp The proportional coefficient.
        /// @param ki The integral coefficient.
        /// @param kd The derivative coefficient.
        /// @param kf The feedforward coefficient.
        /// @param period The period between controller updates in seconds. Must be non-zero and positive.
        inline PIDFController(double kp, double ki, double kd, double kf, double period) : PIDController(kp, ki, kd, period) {
            SetF(kf);
        }

        /// Get the Feedforward coefficient.
        /// @return feedforward coefficient
        constexpr double GetF() const {
            return kF;
        }

        /// Sets the Feedforward coefficient of the PIDF controller gain.
        /// @param kf feedforward coefficient
        constexpr void SetF(double kf) {
            this->kF = kf;
        }

        /// Sets the PIDF Controller gain parameters.
        /// <p>Set the proportional, integral, differential, and feedforward coefficients.
        /// @param kp The proportional coefficient.
        /// @param ki The integral coefficient.
        /// @param kd The derivative coefficient.
        /// @param kf The feedforward coefficient.
        inline void SetPIDF(double kp, double ki, double kd, double kf) {
            SetPID(kp, ki, kd);
            SetF(kf);
        }

        /// Returns the next output of the PID controller.
        /// @param measurement The current measurement of the process variable.
        /// @return The next controller output.
        inline double Calculate(double measurement) {
            return PIDController::Calculate(measurement) + measurement * kF;
        }

        inline virtual void InitSendable(wpi::SendableBuilder& builder) {
            PIDController::InitSendable(builder);
            builder.AddDoubleProperty("f", [this] { return GetF(); }, [this](double value) { SetF(value); });
        }
    };
};