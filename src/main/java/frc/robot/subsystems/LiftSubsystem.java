/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {
  /**
   * Creates a new LiftSubsystem.
   */
  private VictorSP liftMotor = new VictorSP(Constants.LiftConstants.liftMotor);

  public LiftSubsystem() {

  }

  public void liftSpeed(double power){
    liftMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
