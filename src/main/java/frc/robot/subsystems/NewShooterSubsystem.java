/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NewShooterSubsystem extends SubsystemBase {

	private WPI_TalonFX shooterMotorOne = new WPI_TalonFX(Constants.NewShooterConstants.shooterOne);
	private WPI_TalonFX shooterMotorTwo = new WPI_TalonFX(Constants.NewShooterConstants.shooterTwo);
	private WPI_TalonSRX feedMotor = new WPI_TalonSRX(Constants.NewShooterConstants.feedMotor);

	public NewShooterSubsystem() {
		// Dont't allow the power to be X instantly, make it
		//  slowly adjust to the target speed over time.
		shooterMotorOne.configOpenloopRamp(1);
		shooterMotorTwo.configOpenloopRamp(1);

	}

	/**
	 * Set the Speed for the motors
	 *
	 * @param rearMotor
	 * @param frontMotor
	 */
	public void setSpeed(double rearMotor, double frontMotor, double feed) {
		shooterMotorOne.set(rearMotor);
		shooterMotorTwo.set(frontMotor);
		feedMotor.set(feed);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

}
