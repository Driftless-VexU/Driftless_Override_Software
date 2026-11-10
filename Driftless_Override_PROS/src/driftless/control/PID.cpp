#include "driftless/control/PID.hpp"

namespace driftless {
namespace control {
PID::PID(std::unique_ptr<driftless::rtos::IClock> clock, double kp, double ki,
         double kd)
    : m_clock{std::move(clock)}, m_kp{kp}, m_ki{ki}, m_kd{kd} {}

PID::PID(const PID& other)
    : m_clock{other.m_clock->clone()},
      m_kp{other.m_kp},
      m_ki{other.m_ki},
      m_kd{other.m_kd},
      accumulated_error{other.accumulated_error},
      last_error{other.last_error},
      last_time{other.last_time} {}

double PID::getControlValue(double current, double target, double max) {
  // update the time
  double time_change{};
  if (m_clock) {
    double current_time = m_clock->getTime();
    time_change = current_time - last_time;
    last_time = current_time;
  }

  // update error
  double error{target - current};

  double error_change{(error - last_error) / time_change};
  last_error = error;

  // calc control value
  double new_accumulated = accumulated_error + error * time_change;

  if (m_ki > 0) {
    new_accumulated = std::clamp(new_accumulated, -max / m_ki, max / m_ki);
  }

  double unclamped =
      m_kp * error + m_ki * new_accumulated + m_kd * error_change;

  bool pushing_deeper =
      (unclamped > max && error > 0) || (unclamped < -max && error < 0);
  if (!pushing_deeper) {
    accumulated_error = new_accumulated;
  }

  double output = m_kp * error + m_ki * accumulated_error + m_kd * error_change;
  return std::clamp(output, -max, max);
}

void PID::reset() {
  if (m_clock) {
    last_time = m_clock->getTime();
  }
  accumulated_error = 0;
  last_error = 0;
}

PID& PID::operator=(const PID& rhs) {
  m_clock = rhs.m_clock->clone();
  m_kp = rhs.m_kp;
  m_ki = rhs.m_ki;
  m_kd = rhs.m_kd;
  accumulated_error = rhs.accumulated_error;
  last_error = rhs.last_error;
  last_time = rhs.last_time;
  return *this;
}
}  // namespace control
}  // namespace driftless