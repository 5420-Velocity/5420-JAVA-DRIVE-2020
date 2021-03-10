package frc.robot;

import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// This will allow a function to be executed
// with the options to run untill the output is true.
public class RunnableCommand extends CommandBase {

	private boolean isFinished;
	private BooleanSupplier daFunc;
	private Integer timeout;
	private Date timeoutDeadline;

	public RunnableCommand(Runnable run) {
		this(() -> {
			run.run();
			return true;
		}, null);
	}

	public RunnableCommand(Runnable run, Integer timeout) {
		this(() -> {
			run.run();
			return true;
		}, timeout);
	}

	public RunnableCommand(BooleanSupplier daFunc) {
		this(daFunc, null);
	}

	public RunnableCommand(BooleanSupplier daFunc, Integer timeout) {
		this.daFunc = daFunc;
		this.timeout = timeout;
	}

	@Override
	public void initialize() {
		this.isFinished = false;

		if (this.timeout != null) {	
			Calendar deadlineCalculate = GregorianCalendar.getInstance();
			deadlineCalculate.add(GregorianCalendar.MILLISECOND, this.timeout);
			this.timeoutDeadline = deadlineCalculate.getTime();
		}
	}

	@Override
	public void execute() {
		this.isFinished = this.daFunc.getAsBoolean();
	}

	@Override
	public boolean isFinished() {

		if (this.timeout != null && (new Date()).after(this.timeoutDeadline)) {
			System.out.println("RunnableCommand::execute Command Deadline Reached");
			return true;
		}

		return this.isFinished;
	}

}