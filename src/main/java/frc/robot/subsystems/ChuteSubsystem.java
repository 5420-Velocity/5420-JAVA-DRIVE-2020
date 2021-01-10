/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ChuteSubsystem extends SubsystemBase {
	/**
	 * Creates a new Chute.
	 */
	public WPI_TalonSRX chuteLeft = new WPI_TalonSRX(Constants.ChuteConstants.CAN.LeftChute);
	public WPI_TalonSRX chuteRight = new WPI_TalonSRX(Constants.ChuteConstants.CAN.RightChute);

	public ChuteSubsystem() {

	}

	public void setSpeed(double power) {
		chuteLeft.set(power);
		chuteRight.set(-power);
	}

	public void setLeft(double power) {
		chuteLeft.set(power);
	}

	public void setRight(double power) {
		chuteRight.set(power);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
