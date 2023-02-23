#define BUTTON_L(id) \
  [&] { return Robot::GetRobot()->GetButtonBoard().GetRawButton(id); }