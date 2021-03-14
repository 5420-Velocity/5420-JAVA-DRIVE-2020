/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Color;
import io.github.pseudoresonance.pixy2api.links.Link;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final class AutoConstants{
		public static final double kRamseteB = 0;
		public static final double kRamseteZeta = 0;
	}

	public static final class IntakeConstants {

		public static final double highTarget = 7; // Encoder Position for high position
		public static final double middleTarget = 7; // Encoder position for middle position
		public static final double lowTarget = 25.5; // Encoder Position for low position

		// PID Control Values
		public static final double Proportional = 0.08;
		public static final double Integral = 0.04;
		public static final double Derivative = 0.005;

		public static final class DIO {
			public static final int encoderPort = 0;
		}

		public static final class PWM {
			public static final int armMotor = 11;
			public static final int intakeMotor = 9;
		}

		public static final Link pixyLink = new SPILink();
		public static final SPI.Port pixyLinkPort = SPI.Port.kOnboardCS0;
	}

	public static final class ShooterConstants {

		// Target range for the limelight to drive the bot
		public static final double rangeGoal = 120;
		public static final double knownDistance = 110.0;
		public static final double knownArea = 2.075;

		// If you are having trouble figuring out what the angle a1 is,
		//  you can also use the above equation to solve for a1.
		// Just put your robot at a known distance (measuring from the
		//  lens of your camera) and solve the same equation for a1.
		public static final double h1 = 11.625;
		public static final double h2 = 90;
		public static final double a1 = 35;

		public static final class CAN {
			public static final int shooterOut = 6;
			public static final int shooterIn = 0;

			public static final int aimerMotor = 13;
		}
	}

	public static final class NewShooterConstants {

		public static final class CAN {
			public static final int shooterOne = 14;
			public static final int shooterTwo = 15;
			public static final int feedMotor = 16;
		}
		public static final int shooterCover = 1;

	}

	public static final class ControlPanelConstants {

		// Timeout wait in milliseconds
		public static final int timeOutTime = 3500;

		// Number of changes in color
		public static final int targetRotations = 32;

		public static final class CAN {
			public static final int ControlPanelDriver = 5;
			public static final int ControlPanelLift = 7;
		}

		
		public static final I2C.Port ColorSensorPort = I2C.Port.kOnboard;

		public static final class DIO {
			// Upper and lower limits for the panel controller lift
			public static final int upperLimit = 2;
			public static final int lowerLimit = 1;
		}

	}

	public static final class LiftConstants {

		public static final class PWM {
			public static final int liftMotor = 3;
		}

	}

	public static final class DriveTrainConstants {

		public static final class CAN {
			// Left Motor Controller Map for the Talon CAN IDs
			public static final int Left_A_ID = 1;
			public static final int Left_B_ID = 2;

			// Right Motor Controller Map for the Talon CAN IDs
			public static final int Right_A_ID = 3;
			public static final int Right_B_ID = 4;
		}

		// Gyro
		public static final SPI.Port Port = SPI.Port.kOnboardCS1;

		// Path values
		public static final double ksVolts = 0;
		public static final double kvVoltSecondsPerMeter = 0;
		public static final double kaVoltSecondsSquaredPerMeter = 0;
		public static final double kPDriveVel = 0;
		public static final DifferentialDriveKinematics kDriveKinematics = null;


		// PID values
		public static final double RangeP = 0.04;
		public static final double RangeI = 0.0;
		public static final double RangeD = 0.00;

		public static final double TurnP = 0.14;
		public static final double TurnI = 0.0;
		public static final double TurnD = 0.01;

		public static final double EncoderP = 0.04;
		public static final double EncoderI = 0.0;
		public static final double EncoderD = 0.0;

		// Trans Solenoid
		public static final int transmission = 0;
		public static final boolean defaultGear = false;

		// Autonomous values
		public static final double FastPower = 0.7; 
		public static final double SlowPower = 0.4;
		public static final double botSpeedAtPower = 0.5/27.5; //0.5 power at 27.5 inches per second.
		public static final double botWidth = 22;
		public static final double FXticksPerInch = 100;

		public static final int LeftA = 1;
		public static final int LeftB = 2;
		public static final int RightA = 3;
		public static final int RightB = 4;
		public static final double TicksPerInch = 104;

	}

	public static final class charlesConstants {

		public static final class CAN {
			public static final int charlesMotor = 750;
		}

		public static final int charlesColorSensor = 752;
		public static final double ticksPerBall = 753;
		public static final double initEncoderValue = 754;

		public static final Slot[] slots = new Slot[]{
			new Slot(0, 0, 10),
			new Slot(1, 10, 20),
			new Slot(2, 20, 30),
			new Slot(3, 30, 40),
			new Slot(4, 40, 50),
		};
	}

	public static final class ColorTargets {

		public static final class I2C {
			public static final Port ColorSensor = Port.kMXP;
		}

		public static final Color COLOR_BLUE = ColorMatch.makeColor(0.143, 0.427, 0.429);
		public static final Color COLOR_GREEN = ColorMatch.makeColor(0.197, 0.561, 0.240);
		public static final Color COLOR_RED = ColorMatch.makeColor(0.561, 0.232, 0.114);
		public static final Color COLOR_YELLOW = ColorMatch.makeColor(0.361, 0.524, 0.113);

		/**
		 * Returns the string representation of the color
		 * instance we have.
		 */
		public static String resolveColor(Color color) {
			if (color == COLOR_BLUE) return "blue";
			if (color == COLOR_GREEN) return "green";
			if (color == COLOR_RED) return "red";
			if (color == COLOR_YELLOW) return "yellow";
			return "unknown";
		}

	}

	public static final class ChuteConstants {

		public static final class CAN {
			public static final int LeftChute = 12;
			public static final int RightChute = 10;
		}

	}

	public static final class NetworkTableEntries {

		public static final String COLOR_VALUE = "Color Sensor Value";
		public static final String COLOR_ENCODER_VALUE = "Color Encoder Value";
		public static final String ENCODER_VALUE = "Encoder";
		public static final String FMSCOLOR_VALUE = "FMS Goal Color";
		public static final String GYRO_VALUE = "Gyro Value";

	}

	public static final class ControllerConstants {

		public static final int JOYSTICK_USB_DRIVER = 0;
		public static final int JOYSTICK_USB_OPERATOR = 1;

	}

	public static final class ControllerMapConstants {

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
		public static final int Right_Trigger = 3;
		public static final int Left_Trigger = 2;

		public static final int Joystick_Left_Button = 9;
		public static final int Joystick_Right_Button = 10;
	}

}
