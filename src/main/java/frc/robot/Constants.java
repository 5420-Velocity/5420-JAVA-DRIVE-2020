/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveTrainConstants {

		// Left Motor Controller Map for the Talon CAN IDs
		public static final int Left_A_ID = 1;
		public static final int Left_B_ID = 2;

		// Right Motor Controller Map for the Talon CAN IDs
		public static final int Right_A_ID = 1;
		public static final int Right_B_ID = 2;

	}

	public static final class ControllerConstants {
		
		public static final int JOYSTICK_USB_DRIVER = 0;
		public static final int JOYSTICK_USB_OPERATOR = 1;

	}
	
	public static final class ButtonMapConstants {

		public static final int JOYSTICK_RIGHT_X_AXIS = 4;
		public static final int JOYSTICK_RIGHT_Y_AXIS = 5;
		public static final int JOYSTICK_LEFT_X_AXIS = 0;
		public static final int JOYSTICK_LEFT_Y_AXIS = 1;

		public static final int Red_Button_ID = 1;
		public static final int Green_Button_ID = 0;
		public static final int Yellow_Button_ID = 3;
		public static final int Blue_Button_ID = 2;

	}

}
