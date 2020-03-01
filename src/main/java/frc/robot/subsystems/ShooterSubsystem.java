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

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  private WPI_TalonSRX aimerMotor = new WPI_TalonSRX(Constants.ShooterConstants.aimerMotor);

  private WPI_TalonSRX shooterOutMotor = new WPI_TalonSRX(Constants.ShooterConstants.shooterOut);
  private WPI_TalonSRX shooterInMotor = new WPI_TalonSRX(Constants.ShooterConstants.shooterIn);

  public ShooterSubsystem() {

  }

  /**
   * 
   * @param power1
   * @param power2
   */
  public void setSpeed(double power1, double power2){
    shooterOutMotor.set(power1);
    shooterInMotor.set(power2);
  }

  public void turnSpeed(double power){
    aimerMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
