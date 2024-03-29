/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.RobotContainer.Side;

/**
 * Add your docs here.
 */
public class DriveTrain extends PIDSubsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private WPI_TalonFX LeftAT = new WPI_TalonFX(DriveTrainConstants.CAN.Left_A_ID);
	private WPI_TalonFX LeftBT = new WPI_TalonFX(DriveTrainConstants.CAN.Left_B_ID);

	private WPI_TalonFX RightAT = new WPI_TalonFX(DriveTrainConstants.CAN.Right_A_ID);
	private WPI_TalonFX RightBT = new WPI_TalonFX(DriveTrainConstants.CAN.Right_B_ID);

	private Solenoid trans = new Solenoid(Constants.DriveTrainConstants.transmission);

	private DifferentialDrive drive;

	private final NetworkTableEntry gyroEntry = NetworkTableInstance.getDefault().getEntry(Constants.NetworkTableEntries.GYRO_VALUE);

	private final Encoder leftEncoder = new Encoder(Constants.DriveTrainConstants.LeftA, Constants.DriveTrainConstants.LeftB);
	private final Encoder rightEncoder = new Encoder(Constants.DriveTrainConstants.RightA, Constants.DriveTrainConstants.RightB);

	public DriveTrain() {
		super(new PIDController(Constants.DriveTrainConstants.LongEncoderP, Constants.DriveTrainConstants.LongEncoderI, Constants.DriveTrainConstants.LongEncoderD));

		this.shift(Constants.DriveTrainConstants.defaultGear);

		LeftAT.configFactoryDefault();
		LeftBT.configFactoryDefault();
		RightAT.configFactoryDefault();
		RightBT.configFactoryDefault();

		setBrakeMode(NeutralMode.Coast);

		LeftAT.configOpenloopRamp(1);
		LeftBT.configOpenloopRamp(1);
		RightAT.configOpenloopRamp(1);
		RightBT.configOpenloopRamp(1);

		LeftBT.follow(LeftAT);
		RightBT.follow(RightAT);

		drive = new DifferentialDrive(LeftAT, RightAT);
	}

	@Override
	public void periodic() {
		super.periodic();

		// Update the odometry in the periodic block
	
		SmartDashboard.putNumber("Left Encoder", this.getLeftEncoderPosition());
		SmartDashboard.putNumber("Right Encoder", this.getRightEncoderPosition());
		SmartDashboard.putNumber("Target", this.getSetpoint());
	}

	public void arcadeDrive(double speed, double rotation) {
		//Thread.dumpStack();
		drive.arcadeDrive(speed, rotation);
	}

	// Need to implement flipped control capability
	// Radius in inches, time in seconds, side is either "Left" or "Right"
	public void leanTime(double radius, double time, Side side){
		double innerCircumference = radius * 2 * 3.14;
		double outerCircumference = (radius + Constants.DriveTrainConstants.botWidth) * 2 * 3.14;

		double innerPower = (innerCircumference / time) * Constants.DriveTrainConstants.botSpeedAtPower;
		double outerPower = (outerCircumference / time) * Constants.DriveTrainConstants.botSpeedAtPower;

		if(side == Side.Left) {
			tankDrive(innerPower, outerPower);
		}
		else if(side == Side.Right) {
			tankDrive(outerPower, innerPower);
		}
	}

	public void leanPower(double radius, double innerPower, Side side)
	{
		double innerSpeed = innerPower / Constants.DriveTrainConstants.botSpeedAtPower;
		double circumference = (radius + Constants.DriveTrainConstants.botWidth)* 2 * Math.PI;
		double time = (radius * 2 * Math.PI) / innerSpeed;
		double outerPower = (circumference / time) * Constants.DriveTrainConstants.botSpeedAtPower;

		if(side == Side.Left){
			tankDrive(innerPower, outerPower);
		}
		else{
			tankDrive(outerPower, innerPower);
		}
	}

	public void tankDrive(double leftSpeed, double rightSpeed) {
		drive.tankDrive(leftSpeed, rightSpeed);
	}

	public void tankDriveInverted(double leftSpeed, double rightSpeed) {
		drive.tankDrive(rightSpeed, leftSpeed);
	}

	public double getLeftEncoderPosition() {
		return leftEncoder.get() / DriveTrainConstants.TicksPerInch;
	}

	public double getRightEncoderPosition() {
		return rightEncoder.get() / DriveTrainConstants.TicksPerInch;
	}

	public void shift(boolean state) {
		// if(state == true){
		// 	setBrakeMode(NeutralMode.Coast);
		// }
		// else{
		// 	setBrakeMode(NeutralMode.Coast);
		// }
		trans.set(state);
	}

	public void setBrakeMode(NeutralMode mode){
		LeftAT.setNeutralMode(mode);
		LeftBT.setNeutralMode(mode);
		RightAT.setNeutralMode(mode);
		RightBT.setNeutralMode(mode);
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
		LeftAT.setSelectedSensorPosition(0, 0, 10);
		RightAT.setSelectedSensorPosition(0, 0, 10);
		leftEncoder.reset();
		rightEncoder.reset();
	}

	/**
	 * Gets the average distance of the two encoders.
	 *
	 * @return the average of the two encoder readings
	 */
	public double getAverageEncoderDistance() {
		return (this.getLeftEncoderPosition() + this.getRightEncoderPosition()) / 2.0;
	}

	/**
	 * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
	 *
	 * @param maxOutput the maximum output to which the drive will be constrained
	 */
	public void setMaxOutput(double maxOutput) {
		drive.setMaxOutput(maxOutput);
	}

	@Override
	protected void useOutput(double outputDrive, double setPoint) {
		this.arcadeDrive(outputDrive, 0);
	}

	public void enable(){
		super.m_enabled = true;
		this.resetEncoders();
	}

	/** Disables the PID control. Sets output to zero. */
	public void disable() {
		m_enabled = false;
		useOutput(0, 0);
	}   

	public boolean onTarget(){
		if(this.getRightEncoderPosition() - getSetpoint() < 1) {
			return true;
		}
		return false;
	}

	@Override
	protected double getMeasurement() {
		return this.getRightEncoderPosition() / Constants.DriveTrainConstants.TicksPerInch;
	}
}



