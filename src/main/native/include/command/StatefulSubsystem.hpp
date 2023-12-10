#pragma once
#include <frc2/command/SubsystemBase.h>
#include <frc/DriverStation.h>

namespace stormkit {
    namespace command {
        template <class T, std::enable_if_t<std::equality_comparable<T>> = 0> class StatefulSubsystem : public frc2::SubsystemBase {
        private:
            T m_current_state;
        public:
            inline StatefulSubsystem(T starting_state) {
                m_current_state = starting_state;
            }

            inline void ManageState() {
                if (DriverStation::IsAutonomous()) {
                    return;
                }

                T new_state = GetState();

                if (new_state != m_current_state) {
                    OnStateExit(m_current_state);
                    OnStateEnter(new_state);
                    m_current_state = new_state;
                }

                OnStateUpdate(m_current_state);
            }

            inline virtual void Periodic() {
                ManageState();
            }

            inline virtual T GetState() {}

            inline virtual void OnStateEnter(T state) {}

            inline virtual void OnStateExit(T state) {}

            inline virtual void OnStateUpdate(T state) {}
        };
    };
};