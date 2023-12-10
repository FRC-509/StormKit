#pragma once
#include <frc/controller/PIDController.h>
#include <wpi/sendable/SendableBuilder.h>

namespace stormkit {
    class PIDFController : public frc::PIDController {
    private:
        double kF = 0.0f;
    public:
        inline PIDFController(double kP, double kI, double kD, double kF) : PIDController(kP, kI, kD) {
            SetF(kF);
        }

        inline double GetF() const {
            return kF;
        }

        inline void SetF(double kF) {
            this->kF = kF;
        }

        inline void SetPIDF(double kP, double kI, double kD, double kF) {
            SetPID(kP, kI, kD);
            SetF(kF);
        }

        inline double Calculate(double measurement) {
            return PIDController::Calculate(measurement) + measurement * kF;
        }

        inline virtual void InitSendable(wpi::SendableBuilder& builder) {
            PIDController::InitSendable(builder);
            builder.AddDoubleProperty("f", [this] { return GetF(); }, [this](double value) { SetF(value); });
        }
    };
};