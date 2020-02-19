/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final class EncoderArm{
		public static final int encoderPort1 = 0;
		public static final int encoderPort2 = 1;

		public static final int highTarget = 0;
		public static final int lowTarget = 5;

		public static final int armMotor = 11;
		public static final int intakeMotor = 9;

		public static final double Proportional = 0.3;
		public static final double Integral = 0.025;
		public static final double Derivative = 0.005;

	}

	public static final class ShooterConstants{
		public static final int motor1 = 6;
		public static final int motor2 = 8;
		
		public static final int aimerMotor = 13;
	}

	public static final class ControlPanelConstants{
		//number of changes in color
		public static final int targetRotations = 24;

		public static final int ControlPanelDriver = 5;
		public static final int ControlPanelLift = 7;
	}

    public static final class DriveTrainConstants {

		// Left Motor Controller Map for the Talon CAN IDs
		public static final int Left_A_ID = 1;
		public static final int Left_B_ID = 2;

		// Right Motor Controller Map for the Talon CAN IDs
		public static final int Right_A_ID = 3;
		public static final int Right_B_ID = 4;

	}

	public static final class ColorTargets {
		
		public static final Color COLOR_BLUE = ColorMatch.makeColor(0.143, 0.427, 0.429);
		public static final Color COLOR_GREEN = ColorMatch.makeColor(0.197, 0.561, 0.240);
		public static final Color COLOR_RED = ColorMatch.makeColor(0.561, 0.232, 0.114);
		public static final Color COLOR_YELLOW = ColorMatch.makeColor(0.361, 0.524, 0.113);


		/**
		 * Retuns the string representation of the color
		 *  instance we have.
		 * 
		 */
		public static String resolveColor(Color color) {
			if(color == COLOR_BLUE) return "blue";
			if(color == COLOR_GREEN) return "green";
			if(color == COLOR_RED) return "red";
			if(color == COLOR_YELLOW) return "yellow";
			return "unknown";
		}

	}

	public static final class ChuteConstanst {
		public static final int LeftChute = 10;
		public static final int RightChute = 12;
	}

	public static final class NetworkTableEntries {

		public static final String COLOR_VALUE = "Color Sensor Value";
		public static final String ENCODER_VALUE = "Encoder";

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

		public static final int Red_Button_ID = 2;
		public static final int Green_Button_ID = 1;
		public static final int Yellow_Button_ID = 4;
		public static final int Blue_Button_ID = 3;
		
		public static final int Left_Bumper = 5;
		public static final int Right_Bumper = 6;

	}

}
