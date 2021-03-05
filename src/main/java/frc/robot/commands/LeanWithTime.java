/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Date;
import java.util.Calendar;
import java.util.GregorianCalendar;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.RobotContainer.Side;

public class LeanWithTime extends CommandBase {
  
  private final DriveTrain driveTrain;
  private final int duration;
  private final Side side;
  private final double power;

  private boolean isFinished = false;
	private Date completedAt;

  public LeanWithTime(DriveTrain Subsystem, int duration, Side side, double innerPower) {
    this.driveTrain = Subsystem;
    this.duration = duration;
    this.side = side;
    this.power = innerPower;

    addRequirements(Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Calendar calculateDate = GregorianCalendar.getInstance();
		calculateDate.add(GregorianCalendar.MILLISECOND, this.duration);
		this.completedAt = calculateDate.getTime();

		this.isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (new Date().after(completedAt)) {
      driveTrain.tankDrive(0, 0);
      this.isFinished = true;
      return;
    }

    if(this.side == Side.Left){
      driveTrain.leanPower(18, power, Side.Left);;
    }
    else if(this.side == Side.Right){
      driveTrain.leanPower(18, power, Side.Right);;
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
