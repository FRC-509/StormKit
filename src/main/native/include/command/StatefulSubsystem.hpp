#include <frc2/command/SubsystemBase.h>
#include <frc/DriverStation.h>

namespace stormkit {
    namespace command {
        template <class T, std::enable_if_t<std::equality_comparable<T>> = 0> class StatefulSubsystem : public SubsystemBase {
        private:
            T m_current_state;
        public:
            StatefulSubsystem(T starting_state) {
                m_current_state = starting_state;
            }

            void ManageState() {
                if (DriverStation::IsAutonomous()) {
                    return;
                }

                T new_state = SetState();

                if (new_state != m_current_state) {
                    OnStateExit(m_current_state);
                    OnStateEnter(new_state);
                    m_current_state = new_state;
                }

                OnStateUpdate(m_current_state);
            }

            virtual void periodic() {
                ManageState();
            }

            virtual T SetState() {}

            virtual void OnStateEnter(T state) {}

            virtual void OnStateExit(T state) {}

            virtual void OnStateUpdate(T state) {}
        };
    };
};