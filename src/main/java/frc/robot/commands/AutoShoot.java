/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.LinkedList;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ChuteSubsystem;
import frc.robot.subsystems.NewShooterSubsystem;

public class AutoShoot extends CommandBase {

	private final ChuteSubsystem shooterSubsystem;
	private final NewShooterSubsystem newShooter;
	private Date speedRampUpTime;
	private LinkedList<Date> shootInterval = new LinkedList<Date>();
	private AtomicReference<Double> speedRef;
	private boolean feeding = false;
	private Date currentDeadline;
	private boolean isFinished = false;

	public AutoShoot(NewShooterSubsystem newShooter, ChuteSubsystem subsystem, double speedValue) {
		this(newShooter, subsystem, new AtomicReference<Double>(speedValue));
	}


	public AutoShoot(NewShooterSubsystem newShooter, ChuteSubsystem subsystem, AtomicReference<Double> speedRef) {
		this.newShooter = newShooter;
		this.shooterSubsystem = subsystem;
		this.speedRef = speedRef;
		addRequirements(subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

		this.isFinished = false;

		int rampUpTime = 1800; // Delay Time
		int feedTime = 700; // On Time
		int feedTimeSpace = 1300; // Off Time
		int ballCount = 3;

		Calendar calculateDate = GregorianCalendar.getInstance();
		calculateDate.add(GregorianCalendar.MILLISECOND, rampUpTime);
		this.speedRampUpTime = calculateDate.getTime();

		int timelinePosition = rampUpTime;

		// Store the times of when to trigger the shooting
		for (int i = 0; i < ballCount; i++) {
			// On
			Calendar calculateDateBall = GregorianCalendar.getInstance();
			timelinePosition += feedTime;
			calculateDateBall.add(GregorianCalendar.MILLISECOND, timelinePosition);
			this.shootInterval.add(calculateDateBall.getTime());

			// Off
			Calendar calculateDateBallFeed = GregorianCalendar.getInstance();
			timelinePosition += feedTimeSpace;
			calculateDateBallFeed.add(GregorianCalendar.MILLISECOND, timelinePosition);
			this.shootInterval.add(calculateDateBallFeed.getTime());
		}
		
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		this.newShooter.setSpeed(this.speedRef.get(), 0);

		if (this.speedRampUpTime != null) {
			// We can run the speed ramp.
			if (new Date().after(this.speedRampUpTime)) {
				this.speedRampUpTime = null;
			}
		}
		else {
			// Speed Ramp Complete, Fire using the feeder

			if (this.currentDeadline == null && this.shootInterval.size() != 0) {
				this.currentDeadline = this.shootInterval.pop();
			}

			if (this.currentDeadline != null && new Date().after(this.currentDeadline)) {
				this.currentDeadline = null;

				if (this.feeding == true) {
					this.feeding = false;
					this.shooterSubsystem.setRight(0.0);

					// Rotinue is Complete, Mark Command is finished.
					if (this.shootInterval.size() == 0) {
						this.isFinished = true;
					}
				}
				else {
					// Not Feeding, Turn the motor on to feed
					this.feeding = true;
					this.shooterSubsystem.setRight(0.7);
				}
			}
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		this.shooterSubsystem.setRight(0);
		this.newShooter.setSpeed(0, 0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return this.isFinished;
	}
}
