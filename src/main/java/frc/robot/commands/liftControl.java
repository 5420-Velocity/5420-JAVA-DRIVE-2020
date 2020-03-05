/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.Intake;

public class liftControl extends CommandBase {
	/**
	 * Creates a new liftControl.
	 */
	private LiftSubsystem liftS;
	private Intake intakeSubsystem;
	private DoubleSupplier inputUp;
	private DoubleSupplier inputDown;



	public liftControl(LiftSubsystem subsystem, Intake intakeSubsystem , DoubleSupplier inputUp, DoubleSupplier inputDown) {
		this.liftS = subsystem;
		this.intakeSubsystem = intakeSubsystem;
		this.inputDown = inputDown;
		this.inputUp = inputUp;
		addRequirements(subsystem);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if(inputUp.getAsDouble() >= 0.1){
			liftS.liftSpeed(inputUp.getAsDouble()*0.5);
			this.intakeSubsystem.forceArmDown(true);
		}
		else if(inputDown.getAsDouble() >= 0.1){
			liftS.liftSpeed(-inputDown.getAsDouble()*0.5);
		}
		else{
			liftS.liftSpeed(0);
		}
		
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
