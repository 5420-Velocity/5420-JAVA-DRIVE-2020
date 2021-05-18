package frc.robot.commands;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utils.DoubleConsumerBooleanSupplier;
import frc.robot.utils.StateList;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

/**
 * PIDCommand that takes in a BooleanSupplier to allow for Command to end.
 * 
 * The default behavior of PIDCommand has been modified slightly to accommodate new features.
 */
public class FinishablePIDCommand extends PIDCommand {

	/**
	 * This ENUM is used to instruct this to provide
	 * the Source value (from the Sensor such an encoder OR a Limelight)
	 * to the completeSupplier Value.
	 */
	public enum ConsumeValueType {
		Source,
		Output,
		Offset
	}

	public static DoubleConsumerBooleanSupplier defaultDcBs = (value) -> false;

	private final DoubleConsumerBooleanSupplier completeSupplier;
	private StateList<Boolean> completedState;
	private final ConsumeValueType valueType;
	private final AtomicReference<Double> atomicSource = new AtomicReference<Double>(0.0);
	private final AtomicReference<Double> atomicOutput = new AtomicReference<Double>(0.0);

	public FinishablePIDCommand(PIDController controller,
			DoubleSupplier measurementSource,
			double setpoint,
			DoubleConsumer useOutput,
			Subsystem... requirements) {
		// Default Signature for PID Controller, Iherihts all default behavior
		this(controller, measurementSource, setpoint, useOutput, FinishablePIDCommand.defaultDcBs, requirements);
	}

	public FinishablePIDCommand(PIDController controller,
			DoubleSupplier measurementSource,
			double setpoint,
			DoubleConsumer useOutput,
			DoubleConsumerBooleanSupplier completeSupplier,
			Subsystem... requirements) {
		// Default Signature but has a "completeSupplier" where it allows for a finishing check
		this(controller, measurementSource, setpoint, useOutput, ConsumeValueType.Offset, FinishablePIDCommand.defaultDcBs, requirements);
	}

	public FinishablePIDCommand(PIDController controller,
			DoubleSupplier measurementSource,
			double setpoint,
			DoubleConsumer useOutput,
			ConsumeValueType type,
			DoubleConsumerBooleanSupplier completeSupplier,
			Subsystem... requirements) {
		super(controller, measurementSource, setpoint, useOutput, requirements);

		requireNonNullParam(completeSupplier, "command", "FinishablePIDCommand");

		// Override the Parent's "useOutput" DoubleConsumer so we can
		//  capture the value so we can cache it to be used later.
		// We store that copied value an atomic value to get later.
		super.m_useOutput = (value) -> {
			this.atomicOutput.set(value);
			useOutput.accept(value);
		};

		super.m_measurement = () -> {
			double value = measurementSource.getAsDouble();
			this.atomicSource.set(value);
			return value;
		};

		this.completeSupplier = completeSupplier;
		this.valueType = type;
	}

	@Override
	public void initialize() {
		super.initialize();

		completedState = StateList.bool(5);

	}
	
	@Override
	public boolean isFinished() {

		if (this.valueType == ConsumeValueType.Output) {
			// Use the Atomic Value we stored earlier to see if this is finished yet.
			this.completedState.add(this.completeSupplier.getAsBoolean(this.atomicOutput.get()));
		}
		else if (this.valueType == ConsumeValueType.Source) {
			// Use the Atomic Value we stored earlier to see if this is finished yet.
			this.completedState.add(this.completeSupplier.getAsBoolean(this.atomicSource.get()));
		}
		else if (this.valueType == ConsumeValueType.Offset) {
			// Use the setPoint and the Atomic Source to get the offset from the target.
			this.completedState.add(this.completeSupplier.getAsBoolean(super.m_setpoint.getAsDouble() - this.atomicSource.get()));
		}

		return this.completedState.get();
	}

}
