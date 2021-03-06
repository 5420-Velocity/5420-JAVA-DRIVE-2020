/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.RobotContainer.Side;

public class LeanWithEncoder extends CommandBase {
  
  private final DriveTrain driveTrain;
  private final double radius;
  private final Side side;
  private final double power;

  private boolean isFinished = false;
  private double target;

  public LeanWithEncoder(DriveTrain Subsystem, double radius, Side side, double innerPower) {
    this.driveTrain = Subsystem;
    this.radius = radius;
    this.side = side;
    this.power = innerPower;

    addRequirements(Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = (radius * 2 * Math.PI) * Constants.DriveTrainConstants.ticksPerInch;

		this.isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.leanPower(this.radius, this.power, this.side);

    if(this.side == Side.Left){
      if(driveTrain.getLeftEncoderPosition() >= target){
        this.isFinished = true;
      }
    }
    else if(this.side == Side.Right){
      if(driveTrain.getRightEncoderPosition() >= target){
        this.isFinished = true;
      }    
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}