package frc.robot.utils;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;

/** A {@link Button} that gets its state from a {@link GenericHID}. */
public class JoystickDPad extends Button {
  private final GenericHID m_joystick;
  private final DPad.Position pos;

  /**
   * Creates a joystick DPad for triggering commands.
   *
   * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
   * @param pos The button number (see {@link GenericHID#getRawButton(int) }
   */
  public JoystickDPad(GenericHID joystick, DPad.Position pos) {
    requireNonNullParam(joystick, "joystick", "JoystickDPad");

    m_joystick = joystick;
    this.pos = pos;
  }

  /**
   * Gets the value of the joystick button.
   *
   * @return The value of the joystick button
   */
  @Override
  public boolean get() {
    return DPad.active(this.m_joystick, pos);
  }
}
