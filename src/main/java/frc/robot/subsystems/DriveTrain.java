/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;

/**
 * Add your docs here.
 */
public class DriveTrain extends SubsystemBase {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private WPI_TalonFX LeftAT = new WPI_TalonFX(DriveTrainConstants.Left_A_ID);
	private WPI_TalonFX LeftBT = new WPI_TalonFX(DriveTrainConstants.Left_B_ID);

	private WPI_TalonFX RightAT = new WPI_TalonFX(DriveTrainConstants.Right_A_ID);
	private WPI_TalonFX RightBT = new WPI_TalonFX(DriveTrainConstants.Right_B_ID);

	private Solenoid trans = new Solenoid(Constants.DriveTrainConstants.transmission);

	private DifferentialDrive drive;

	public DriveTrain() {
		this.shift(Constants.DriveTrainConstants.defaultGear);

		LeftAT.configFactoryDefault();
		LeftBT.configFactoryDefault();
		RightAT.configFactoryDefault();
		RightBT.configFactoryDefault();

		LeftAT.setNeutralMode(NeutralMode.Coast);
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
	}

	public void zero() {
		LeftAT.getSensorCollection().setIntegratedSensorPosition(0, 5000);
		RightAT.getSensorCollection().setIntegratedSensorPosition(0, 5000);
	}

	public void arcadeDrive(double speed, double rotation) {
		drive.arcadeDrive(speed, rotation);
	}

	public double getLeftEncoder() {
		return LeftAT.getSelectedSensorPosition();
	}

	public double getRightEncoder() {
		return RightAT.getSelectedSensorPosition();
	}

	public void shift(boolean state) {
		trans.set(state);
	}

}
