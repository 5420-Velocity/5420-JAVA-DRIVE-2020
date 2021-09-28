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
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FeedMoment;
import frc.robot.FeedMoment.MomentType;
import frc.robot.subsystems.ChuteSubsystem;
import frc.robot.subsystems.NewShooterSubsystem;

public class AutoShoot extends CommandBase {

	private final ChuteSubsystem shooterSubsystem;
	private final NewShooterSubsystem newShooter;
	private Date speedRampUpTime;
	private LinkedList<FeedMoment> shootInterval = new LinkedList<FeedMoment>();
	private DoubleSupplier speedRef;
	private FeedMoment currentDeadline;
	private boolean isFinished = false;

	public AutoShoot(NewShooterSubsystem newShooter, ChuteSubsystem subsystem, double speedValue) {
		this(newShooter, subsystem, () -> speedValue);
	}

	public AutoShoot(NewShooterSubsystem newShooter, ChuteSubsystem subsystem, AtomicReference<Double> speedValue) {
		this(newShooter, subsystem, speedValue::get);
	}

	public AutoShoot(NewShooterSubsystem newShooter, ChuteSubsystem subsystem, DoubleSupplier speedRef) {
		this.newShooter = newShooter;
		this.shooterSubsystem = subsystem;
		this.speedRef = speedRef;
		addRequirements(subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

		this.isFinished = false;
		this.currentDeadline = null;

		int rampUpTime = 500; // Delay Time norm 1800 but I'm getting rid of it ~Jake
		int feedForwardTime = 1200; // Forward Feed Time
		int feedReverseTime = 0; // Reverse Feed Time
		int feedTimeSpace = 300; // Off Time
		int ballCount = 3;

		Calendar calculateDate = GregorianCalendar.getInstance();
		calculateDate.add(GregorianCalendar.MILLISECOND, rampUpTime);
		this.speedRampUpTime = calculateDate.getTime();

		int timelinePosition = rampUpTime;

		// This is needed to set the correct interval.
		Calendar sacrificialInit = GregorianCalendar.getInstance();
		sacrificialInit.add(GregorianCalendar.MILLISECOND, timelinePosition);
		this.shootInterval.add(new FeedMoment(MomentType.Off, sacrificialInit.getTime()));

		// Store the times of when to trigger the shooting
		for (int i = 0; i < ballCount; i++) {
			// Forward
			Calendar calculateDateBallOn = GregorianCalendar.getInstance();
			timelinePosition += feedForwardTime;
			calculateDateBallOn.add(GregorianCalendar.MILLISECOND, timelinePosition);
			this.shootInterval.add(new FeedMoment(MomentType.Forward, calculateDateBallOn.getTime()));

			// Reverse
			Calendar calculateDateBallFeedReverse = GregorianCalendar.getInstance();
			timelinePosition += feedReverseTime;
			calculateDateBallFeedReverse.add(GregorianCalendar.MILLISECOND, timelinePosition);
			this.shootInterval.add(new FeedMoment(MomentType.Reverse, calculateDateBallFeedReverse.getTime()));

			// Off
			Calendar calculateDateBallFeedOff = GregorianCalendar.getInstance();
			timelinePosition += feedTimeSpace;
			calculateDateBallFeedOff.add(GregorianCalendar.MILLISECOND, timelinePosition);
			this.shootInterval.add(new FeedMoment(MomentType.Off, calculateDateBallFeedOff.getTime()));
		}
		
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		this.newShooter.setSpeed(this.speedRef.getAsDouble(), 0);

		if (this.speedRampUpTime != null) {
			this.shooterSubsystem.setRight(0.0);

			// We can run the speed ramp.
			if (new Date().after(this.speedRampUpTime)) {
				this.speedRampUpTime = null;
			}
		}
		else {
			// Speed Ramp Complete, Fire using the feeder

			if (this.currentDeadline == null) {
				// If we dont have a Deadline.

				if (this.shootInterval.size() == 0) {
					// We are out of tasks the Routine is Complete, Mark Command is finished
					this.isFinished = true;
				}
				else {
					// Grab the next Deadline
					this.currentDeadline = this.shootInterval.pop();
				}
			}

			if (this.currentDeadline != null) {
				// We have a deadline

				if (this.currentDeadline.expired()) {
					// If we have a deadline AND it's expired
					this.currentDeadline = null;
				}
				else {
					// If we have a deadline
					if (this.currentDeadline.type == MomentType.Off) {
						this.shooterSubsystem.setRight(0);
					}
					else if (this.currentDeadline.type == MomentType.Forward) {
						this.shooterSubsystem.setRight(0.7);
					}
					else if (this.currentDeadline.type == MomentType.Reverse) {
						this.shooterSubsystem.setRight(-0.4);
					}
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
