package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * DPad
 *
 * @author Noah Halstead <nhalstead00@gmail.com>
 * @author Team 1736 Robot Casserole
 * @link https://github.com/RobotCasserole1736/CasseroleLib/blob/master/java/src/org/usfirst/frc/team1736/lib/HAL/Xbox360Controller.java
 */
public class DPad {

	public enum Position {
		kUp,
		kDown,
		kLeft,
		kRight
	}

	// -Controller D-Pad POV Hat
	final static int XBOX_DPAD_POV = 0;
	public GenericHID inJoystick;

	/**
	 * Return if its value is Up/Down/Left/Right is true.
	 * @param joy
	 * @param pos
	 * @return
	 */
	public static boolean active(GenericHID joy, Position pos) {
		switch(pos) {
			case kUp:
				return DPad.up(joy);
			case kDown:
				return DPad.down(joy);
			case kLeft:
				return DPad.left(joy);
			case kRight:
				return DPad.right(joy);
			default:
				return false;
		}
	}

	/**
	 * Return if its value is Up/Down/Left/Right is true, matching 2 params.
	 * @param joy
	 * @param pos
	 * @param pos2
	 * @return
	 */
	public static boolean active(GenericHID joy, Position pos, Position pos2) {
		return (active(joy, pos) && active(joy, pos2));
	}

	/**
	 * @return The current "angle" from the DPad (POV switch)
	 */
	public static int get(GenericHID joy) {
		return joy.getPOV(XBOX_DPAD_POV);
	}

	/**
	 * Is the DPad button Up
	 *
	 * @return True if the DPad is pushed up, False if it is not pressed
	 */
	public static boolean up(GenericHID joy) {
		return (joy.getPOV(XBOX_DPAD_POV) >= 315 || joy.getPOV(XBOX_DPAD_POV) <= 45) && joy.getPOV(XBOX_DPAD_POV) != -1;
	}


	/**
	 * Is the DPad button on the Right
	 *
	 * @return True if the DPad is pushed right, False if it is not pressed
	 */
	public static boolean right(GenericHID joy) {
		return joy.getPOV(XBOX_DPAD_POV) >= 45 && joy.getPOV(XBOX_DPAD_POV) <= 135;
	}


	/**
	 * Is the DPad button Down
	 *
	 * @return True if the DPad is pushed down, False if it is not pressed
	 */
	public static boolean down(GenericHID joy) {
		return joy.getPOV(XBOX_DPAD_POV) >= 135 && joy.getPOV(XBOX_DPAD_POV) <= 225;
	}


	/**
	 * Is the DPad button on the Left
	 *
	 * @return True if the DPad is pushed left, False if it is not pressed
	 */
	public static boolean left(GenericHID joy) {
		return joy.getPOV(XBOX_DPAD_POV) >= 225 && joy.getPOV(XBOX_DPAD_POV) <= 315;
	}

	public DPad(GenericHID joy) {
		this.inJoystick = joy;
	}

	public boolean up() {
		return DPad.up(this.inJoystick);
	}

	public boolean down() {
		return DPad.down(this.inJoystick);
	}

	public boolean right() {
		return DPad.right(this.inJoystick);
	}

	public boolean left() {
		return DPad.left(this.inJoystick);
	}

	public int get() {
		return DPad.get(this.inJoystick);
	}
}