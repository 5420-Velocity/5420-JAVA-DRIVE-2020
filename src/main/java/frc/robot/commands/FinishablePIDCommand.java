package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

/**
 * PIDComamnd that takes in a BooleanSupplier to allow for Command to end.
 * 
 */
public class FinishablePIDCommand extends PIDCommand {

	private BooleanSupplier completeSupplier;

	public FinishablePIDCommand(PIDController controller, DoubleSupplier measurementSource, double setpoint,
			DoubleConsumer useOutput, Subsystem... requirements) {
		super(controller, measurementSource, setpoint, useOutput, requirements);

		this.completeSupplier = () -> false;
	}

	public FinishablePIDCommand(PIDController controller, DoubleSupplier measurementSource, double setpoint,
			DoubleConsumer useOutput, BooleanSupplier completeSupplier, Subsystem... requirements) {
		super(controller, measurementSource, setpoint, useOutput, requirements);

		requireNonNullParam(completeSupplier, "command", "FinishablePIDCommand");

		this.completeSupplier = completeSupplier;
	}
	
	@Override
	public boolean isFinished() {
		return this.completeSupplier.getAsBoolean();
	}

}
