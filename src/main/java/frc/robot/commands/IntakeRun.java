/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeRun extends CommandBase {

  private Intake intake;
  private BooleanSupplier Input;

  public IntakeRun(Intake intakeRun, BooleanSupplier input) {
    this.Input = input;
    this.intake = intakeRun;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Input.getAsBoolean()){
      if(intake.getEncoderValue() <=5 && intake.getEncoderValue() >= 0){
        intake.ArmRun(0.8);
        intake.IntakeMove(0.8);
      }
        else{
          intake.ArmRun(0);
          intake.IntakeMove(0);
        }
    }
      else if(intake.getEncoderValue() <=5 && intake.getEncoderValue() >= 0){
        intake.ArmRun(-0.5);
        intake.IntakeMove(0);
      }
        else{
          intake.ArmRun(0);
          intake.IntakeMove(0);
        }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
