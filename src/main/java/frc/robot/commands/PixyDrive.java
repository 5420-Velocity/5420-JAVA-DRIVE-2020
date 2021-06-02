// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BlockExtra;
import frc.robot.PixyAlgo;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class PixyDrive extends CommandBase {
  private final DriveTrain driveTrain;
  private final Intake subsystem;
  private double motorSpeed;
  private double targetArea = 180;
  private boolean isFinished;

  /** Creates a new PixieDrive. */
  public PixyDrive(Intake subsystem, DriveTrain driveTrain) {
    this.subsystem = subsystem;
    this.driveTrain = driveTrain;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    motorSpeed = 0;
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(subsystem.pixyAlgo.getPixyBest() != null) {
      if(subsystem.pixyAlgo.getPixyBest().getWidth() < targetArea) {
        driveTrain.arcadeDrive(0.6, 0);
      }
      else {
        driveTrain.arcadeDrive(0, 0);
        isFinished = true;
      }
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