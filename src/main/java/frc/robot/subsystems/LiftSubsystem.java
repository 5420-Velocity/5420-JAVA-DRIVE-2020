/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {
	/**
	 * Creates a new LiftSubsystem.
	 */
	private WPI_TalonSRX liftMotor = new WPI_TalonSRX(Constants.LiftConstants.CAN.liftMotor);

	public LiftSubsystem() {
		liftMotor.setNeutralMode(NeutralMode.Brake);
	}

	public void liftSpeed(double power) {
		liftMotor.set(power);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
