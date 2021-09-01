/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class PixySearch extends CommandBase {
  /**
   * Creates a new PixySearch.
   */
  private Intake Intake;
  private DriveTrain DriveTrain;
  private boolean Clockwise;
  private boolean isFinished;
  public PixySearch(Intake intake, DriveTrain driveTrain) {
    this.Intake = intake;
    this.DriveTrain = driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isFinished = false;
    if(Intake.pixyAlgo.secondaryClockwise != null && Intake.pixyAlgo.secondaryClockwise){
      Clockwise = true;
    }
    else{
      Clockwise = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Turn until pixy has a target
    if(Intake.pixyAlgo.getPixyBest() != null){
      this.isFinished = true;
    }
    else{
      if(Clockwise){
        DriveTrain.arcadeDrive(0, -0.4);
      }
      else{
        DriveTrain.arcadeDrive(0, 0.4);
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}
