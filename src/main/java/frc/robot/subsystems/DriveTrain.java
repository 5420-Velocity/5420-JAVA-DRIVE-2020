/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
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

	// Odometry class for tracking robot pose
	private final DifferentialDriveOdometry m_odometry;

	// The gyro sensor
	private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(Constants.DriveTrainConstants.Port);

	public DriveTrain() {
		this.shift(Constants.DriveTrainConstants.defaultGear);

		LeftAT.configFactoryDefault();
		LeftBT.configFactoryDefault();
		RightAT.configFactoryDefault();
		RightBT.configFactoryDefault();

		LeftAT.setNeutralMode(NeutralMode.Coast);
		LeftBT.setNeutralMode(NeutralMode.Coast);
		RightAT.setNeutralMode(NeutralMode.Coast);
		RightBT.setNeutralMode(NeutralMode.Coast);

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
		m_odometry.update(
			m_gyro.getRotation2d (),
			this.getLeftEncoderPosition(),
			this.getRightEncoderPosition()
		);
	}

	public void zero() {
		LeftAT.getSensorCollection().setIntegratedSensorPosition(0, 800);
		RightAT.getSensorCollection().setIntegratedSensorPosition(0, 800);
	}

	public void arcadeDrive(double speed, double rotation) {
		drive.arcadeDrive(speed, rotation);
	}

	public void tankDrive(double leftSpeed, double rightSpeed) {
		drive.tankDrive(leftSpeed, rightSpeed);
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
		// return new DifferentialDriveWheelSpeeds(0, 0);
		return new DifferentialDriveWheelSpeeds(
			this.LeftAT.getSensorCollection().getIntegratedSensorVelocity(),
			this.RightAT.getSensorCollection().getIntegratedSensorVelocity()
		);
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
		
	}

	/**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
	// return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
	return 0;
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



