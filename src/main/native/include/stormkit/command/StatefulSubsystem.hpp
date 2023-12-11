#pragma once
#include <frc2/command/SubsystemBase.h>
#include <frc/DriverStation.h>

namespace stormkit {
    namespace command {
        /// An alternative way of organizing subsystem-specific code.
        /// This generic class takes in an enum that represents the state of the subsytem, and provides interfaces for triggering code when states are entered and exited.
        template <class T, std::enable_if_t<std::equality_comparable<T>> = 0> class StatefulSubsystem : public frc2::SubsystemBase {
        protected:
            T m_current_state;
        public:
            /// Constructs an StatefulSubsystem with the given initial state.
            /// @param starting_state The initial state of the subsystem. 
            inline StatefulSubsystem(T starting_state) {
                m_current_state = starting_state;
            }

            /// Detects state changes and calls the enter/exit callbacks. This is called from the subsytem's Periodic() method.
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

            /// Returns the next state, implement this to facilitate state changes. This is called periodically in the ManageState() method.
            /// @return The next state.
            virtual T GetState() = 0;

            /// Called once as the subsystem's state is changed. This is called when a new state is entered in the ManageState() method.
            /// @param state  The new state.
            virtual void OnStateEnter(T state) = 0;

            /// Called once as the subsystem's state is changed. This is called when a state is exited in the ManageState() method.
            /// @param state  The old state.
            virtual void OnStateExit(T state) = 0;

            /// This is called periodically in the ManageState() method.
            /// @param state  The current state.
            virtual void OnStateUpdate(T state) = 0;
        };
    };
};