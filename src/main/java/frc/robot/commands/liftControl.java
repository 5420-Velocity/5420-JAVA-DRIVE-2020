/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;

public class liftControl extends CommandBase {
	/**
	 * Creates a new liftControl.
	 */
	private LiftSubsystem liftS;
	private BooleanSupplier upButton;
	private BooleanSupplier downButton;


	public liftControl(LiftSubsystem subsystem, BooleanSupplier UpButton, BooleanSupplier DownButton) {
		this.liftS = subsystem;
		this.upButton = UpButton;
		this.downButton = DownButton;
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
		if(upButton.getAsBoolean()){
		liftS.liftSpeed(0.95);
		}
		else if(downButton.getAsBoolean()){
		liftS.liftSpeed(-0.95);
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
