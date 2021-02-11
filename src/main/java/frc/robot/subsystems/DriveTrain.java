/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;

/**
 * Add your docs here.
 */
public class DriveTrain extends SubsystemBase {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private WPI_TalonFX LeftAT = new WPI_TalonFX(DriveTrainConstants.CAN.Left_A_ID);
	private WPI_TalonFX LeftBT = new WPI_TalonFX(DriveTrainConstants.CAN.Left_B_ID);

	private WPI_TalonFX RightAT = new WPI_TalonFX(DriveTrainConstants.CAN.Right_A_ID);
	private WPI_TalonFX RightBT = new WPI_TalonFX(DriveTrainConstants.CAN.Right_B_ID);

	private Solenoid trans = new Solenoid(Constants.DriveTrainConstants.transmission);

	private DifferentialDrive drive;

	// The left-side drive encoder
	private final Encoder m_leftEncoder =
	new Encoder(DriveTrainConstants.Left_A_Encoder, DriveTrainConstants.Left_B_Encoder,
				DriveTrainConstants.L_Reversed);

// The right-side drive encoder
private final Encoder m_rightEncoder =
	new Encoder(DriveTrainConstants.Right_A_Encoder, DriveTrainConstants.Right_B_Encoder,
				DriveTrainConstants.R_Reversed);

	// Odometry class for tracking robot pose
	private final DifferentialDriveOdometry m_odometry;

	// The gyro sensor
	private final PigeonIMU m_gyro = new PigeonIMU(Constants.DriveTrainConstants.CAN.gyro);

	public DriveTrain() {
		this.shift(Constants.DriveTrainConstants.defaultGear);

		LeftAT.configFactoryDefault();
		LeftBT.configFactoryDefault();
		RightAT.configFactoryDefault();
		RightBT.configFactoryDefault();

		LeftAT.setNeutralMode(NeutralMode.Brake);
		LeftBT.setNeutralMode(NeutralMode.Brake);
		RightAT.setNeutralMode(NeutralMode.Brake);
		RightBT.setNeutralMode(NeutralMode.Brake);

		LeftAT.configOpenloopRamp(1);
		LeftBT.configOpenloopRamp(1);
		RightAT.configOpenloopRamp(1);
		RightBT.configOpenloopRamp(1);

		LeftBT.follow(LeftAT);
		RightBT.follow(RightAT);

		drive = new DifferentialDrive(LeftAT, RightAT);
		m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
	}

		@Override
	public void periodic() {
		// Update the odometry in the periodic block
		m_odometry.update(m_gyro.getRotation2d (), m_leftEncoder.getDistance(),
						m_rightEncoder.getDistance());
	}

	public void zero() {
		LeftAT.getSensorCollection().setIntegratedSensorPosition(0, 5000);
		RightAT.getSensorCollection().setIntegratedSensorPosition(0, 5000);
	}

	public void arcadeDrive(double speed, double rotation) {
		drive.arcadeDrive(speed, rotation);
	}

	public double getLeftEncoderPosition() {
		return LeftAT.getSelectedSensorPosition();
	}

	public double getRightEncoderPosition() {
		return RightAT.getSelectedSensorPosition();
	}

	public void shift(boolean state) {
		trans.set(state);
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

		/**
	 * Returns the current wheel speeds of the robot.
	 *
	 * @return The current wheel speeds.
	 */
	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
	}

		/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		resetEncoders();
		m_odometry.resetPosition(pose, m_gyro.getRotation2d());
	}

	public double tankDriveVolts(double left, double right) {
		this.LeftAT.setVoltage(left);
		this.RightAT.setVoltage(right);
		return 0;
	}

	
	/**
	 * Resets the drive encoders to currently read a position of 0.
	 */
	public void resetEncoders() {
		m_leftEncoder.reset();
		m_rightEncoder.reset();
	}

	/**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}



