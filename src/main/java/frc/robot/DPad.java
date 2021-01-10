package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * DPad
 *
 * @author Noah Halstead <nhalstead00@gmail.com>
 * @author Team 1736 Robot Casserole
 * @link https://github.com/RobotCasserole1736/CasseroleLib/blob/master/java/src/org/usfirst/frc/team1736/lib/HAL/Xbox360Controller.java
 */
public class DPad {

	// -Controller D-Pad POV Hat
	final static int XBOX_DPAD_POV = 0;
	public Joystick inJoystick;

	/**
	 * @return The current "angle" from the DPad (POV switch)
	 */
	public static int get(Joystick joy) {
		return joy.getPOV(XBOX_DPAD_POV);
	}


	/**
	 * Is the DPad button Up
	 *
	 * @return True if the DPad is pushed up, False if it is not pressed
	 */
	public static boolean up(Joystick joy) {
		if ((joy.getPOV(XBOX_DPAD_POV) >= 315 || joy.getPOV(XBOX_DPAD_POV) <= 45) && joy.getPOV(XBOX_DPAD_POV) != -1) {
			return true;
		}
		else {
			return false;
		}
	}


	/**
	 * Is the DPad button on the Right
	 *
	 * @return True if the DPad is pushed right, False if it is not pressed
	 */
	public static boolean right(Joystick joy) {
		if (joy.getPOV(XBOX_DPAD_POV) >= 45 && joy.getPOV(XBOX_DPAD_POV) <= 135) {
			return true;
		}
		else {
			return false;
		}
	}


	/**
	 * Is the DPad button Down
	 *
	 * @return True if the DPad is pushed down, False if it is not pressed
	 */
	public static boolean down(Joystick joy) {
		if (joy.getPOV(XBOX_DPAD_POV) >= 135 && joy.getPOV(XBOX_DPAD_POV) <= 225) {
			return true;
		}
		else {
			return false;
		}
	}


	/**
	 * Is the DPad button on the Left
	 *
	 * @return True if the DPad is pushed left, False if it is not pressed
	 */
	public static boolean left(Joystick joy) {
		if (joy.getPOV(XBOX_DPAD_POV) >= 225 && joy.getPOV(XBOX_DPAD_POV) <= 315) {
			return true;
		}
		else {
			return false;
		}
	}

	public DPad(Joystick joy) {
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