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

public class ShooterSubsystem extends SubsystemBase {

	// Used to move the tunnel up or down to aim tward the goal from the LL values.
	private final WPI_TalonSRX aimerMotor = new WPI_TalonSRX(Constants.ShooterConstants.CAN.aimerMotor);

	// This motor is taking the balls from Charlie (the hopper) and pushes to the Front Motor
	private final WPI_TalonSRX shooterMotorBack = new WPI_TalonSRX(Constants.ShooterConstants.CAN.shooterOut);

	// This Motor is used to give the ball extra power to throw it.
	private final WPI_TalonFX shooterMotorFront = new WPI_TalonFX(Constants.ShooterConstants.CAN.shooterIn);

	public ShooterSubsystem() {
		// Don't allow the power to be X instantly, make it
		//  slowly adjust to the target speed over time.
		shooterMotorFront.configOpenloopRamp(1);
		
	}

	/**
	 * Set the Speed for the motors
	 *
	 * @param rearMotor Speed Value for the Back Motor
	 * @param frontMotor Speed Value for the Front Motor
	 */
	public void setSpeed(double rearMotor, double frontMotor) {
		shooterMotorBack.set(rearMotor);
		shooterMotorFront.set(frontMotor);
	}

	public void turnSpeed(double power) {
		aimerMotor.set(power);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

}
