// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class PixyTurn extends CommandBase {
  private final DriveTrain driveTrain;
  private final Intake subsystem;
  private double targetX = 180;
  private boolean isFinished;

  /** Creates a new PixieDrive. */
  public PixyTurn(Intake subsystem, DriveTrain driveTrain) {
    this.subsystem = subsystem;
    this.driveTrain = driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(subsystem.pixyAlgo.getPixyBest() != null) {
      if(subsystem.pixyAlgo.getPixyBest().getX() < (targetX - 20)) {
        driveTrain.arcadeDrive(0, 0.4);
      }
      else if(subsystem.pixyAlgo.getPixyBest().getX() > (targetX + 20)){
        driveTrain.arcadeDrive(0, -0.4);
      }
      else {
        System.out.println("Done");
        driveTrain.arcadeDrive(0, 0);
        isFinished = true;
      }
    }
    else {
      // No blocks detected
      System.out.println("No target...");
      driveTrain.arcadeDrive(0, 0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}
