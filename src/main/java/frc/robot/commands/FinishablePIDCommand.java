package frc.robot.commands;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utils.DoubleConsumerBooleanSupplier;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

/**
 * PIDComamnd that takes in a BooleanSupplier to allow for Command to end.
 * 
 */
public class FinishablePIDCommand extends PIDCommand {

	private DoubleConsumerBooleanSupplier completeSupplier;
	private AtomicReference<Double> atomicOutput = new AtomicReference<Double>();

	public FinishablePIDCommand(PIDController controller, DoubleSupplier measurementSource, double setpoint,
			DoubleConsumer useOutput, Subsystem... requirements) {
		super(controller, measurementSource, setpoint, useOutput, requirements);

		this.completeSupplier = (Double value) -> false;
	}

	public FinishablePIDCommand(PIDController controller, DoubleSupplier measurementSource, double setpoint,
			DoubleConsumer useOutput, DoubleConsumerBooleanSupplier completeSupplier, Subsystem... requirements) {
		super(controller, measurementSource, setpoint, useOutput, requirements);

		requireNonNullParam(completeSupplier, "command", "FinishablePIDCommand");

		this.completeSupplier = completeSupplier;
	}

	@Override
	public void execute() {
		this.atomicOutput.set(
			m_controller.calculate(m_measurement.getAsDouble(), m_setpoint.getAsDouble())
		);
	  m_useOutput.accept(this.atomicOutput.get());
	}
	
	@Override
	public boolean isFinished() {
		return this.completeSupplier.getAsBoolean(this.atomicOutput.get());
	}

}
