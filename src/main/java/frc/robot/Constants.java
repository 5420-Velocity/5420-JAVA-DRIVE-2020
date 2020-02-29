/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2.LinkType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final class IntakeConstants {
		public static final int encoderPort = 0; // Encoder port

		public static final double highTarget = 57.5; // Encoder Position for high position
		public static final double middleTarget = 66; // Encoder position for middle position
		public static final double lowTarget = 75.0; // Encoder Position for low position

		public static final int armMotor = 11; // PID Port
		public static final int intakeMotor = 9; // PID Port

		// PID Control Values
		public static final double Proportional = 0.1;
		public static final double Integral = 0.06;
		public static final double Derivative = 0.005;

		// Define the way to communicate with the Pixy2 Device
		public static final LinkType pixyLinkType = LinkType.I2C;
		public static final int pixyLinkPort = Pixy2.PIXY_DEFAULT_ARGVAL;

	}

	public static final class ShooterConstants {
		public static final int motor1 = 6;
		public static final int motor2 = 8;
		
		public static final int aimerMotor = 13;

	}

	public static final class ControlPanelConstants {
		// Number of changes in color
		public static final int targetRotations = 24;
		public static final int ControlPanelDriver = 5;
		public static final int ControlPanelLift = 7;
	}

	public static final class LiftConstants {
		public static final int liftMotor = 3;
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
		public static final int LeftChute = 12;
		public static final int RightChute = 10;
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
