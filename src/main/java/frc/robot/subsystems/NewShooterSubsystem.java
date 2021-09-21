/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NewShooterSubsystem extends SubsystemBase {

	private final WPI_TalonFX shooterMotorOne = new WPI_TalonFX(Constants.NewShooterConstants.CAN.shooterOne);
	private final WPI_TalonFX shooterMotorTwo = new WPI_TalonFX(Constants.NewShooterConstants.CAN.shooterTwo);
	private final WPI_TalonSRX feedMotor = new WPI_TalonSRX(Constants.NewShooterConstants.CAN.feedMotor);
	private final Solenoid shooterCover = new Solenoid(Constants.NewShooterConstants.shooterCover);

	public enum coverState{
		Up, 
		Down;
	}

	public NewShooterSubsystem() {
		// Don't allow the power to be X instantly, make it
		//  slowly adjust to the target speed over time.
		shooterMotorOne.configOpenloopRamp(1);
		shooterMotorTwo.configOpenloopRamp(1);
		shooterMotorTwo.setInverted(true);
		SmartDashboard.setDefaultNumber("shooterSpeed", 0);
	}

	/**
	 * Set the Speed for the motors
	 *
	 * @param rearMotor Speed Value for the Back Motor
	 * @param frontMotor Speed Value for the Front Motor
	 * @param feed Motor Speed for the Feed   
	 */
	public void setSpeed(double motorSpeed, double feed) {
		shooterMotorOne.set(motorSpeed);
		shooterMotorTwo.set(motorSpeed); // This motor is inverted above
		SmartDashboard.putNumber("shooterSpeed", motorSpeed);
		feedMotor.set(feed);
	}

	public void coverSet(coverState state){
		if(state == coverState.Up) {
			shooterCover.set(true);
		}
		else if(state == coverState.Down){
			shooterCover.set(false);
		}
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

}
