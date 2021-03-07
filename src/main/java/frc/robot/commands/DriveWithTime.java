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

public class DriveWithTime extends CommandBase {

  private final DriveTrain driveTrain;
  private final double distance;
  private final double Power;

  private boolean isFinished = false;
	private Date completedAt;

  public DriveWithTime(DriveTrain Subsystem, double distance, double power) {
    this.driveTrain = Subsystem;
    this.distance = distance;
    this.Power = power;

    addRequirements(Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    double speed = Power / Constants.DriveTrainConstants.botSpeedAtPower;
    double duration = distance / speed;

    Calendar calculateDate = GregorianCalendar.getInstance();
		calculateDate.add(GregorianCalendar.MILLISECOND, (int)duration);
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

    driveTrain.tankDrive(Power, Power);
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
