
package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;

/**
 * This will publish two values to allow the drivers to update delay values for Auto.
 *
 *  X Duration (Read/Write)
 *  x Finished (Readonly)
 *
 */
public class ShuffleboardDelay extends CommandBase {

	private boolean isFinished;
	private Date finishingTime;
	private final NetworkTableEntry completed;
	private final NetworkTableEntry duration;

	public ShuffleboardDelay(String keyname, double defaultDelaySeconds) {
		this.completed = SmartDashboard.getEntry(keyname + " Finished");
		this.duration = SmartDashboard.getEntry(keyname + " Duration");

		this.completed.setDefaultBoolean(false);
		this.duration.setDefaultNumber(defaultDelaySeconds);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		Calendar calculateDate = GregorianCalendar.getInstance();
		calculateDate.add(GregorianCalendar.SECOND, this.duration.getNumber(0).intValue());

		this.finishingTime = calculateDate.getTime();

		this.completed.setBoolean(false);

		this.isFinished = false;
	}

	@Override
	public void execute() {
		if (new Date().after(this.finishingTime)) {
			this.isFinished = true;
			this.completed.setBoolean(true);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
      this.isFinished = true;
      this.completed.setBoolean(true);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return this.isFinished;
	}
}
