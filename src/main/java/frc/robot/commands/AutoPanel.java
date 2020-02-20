/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;

import frc.robot.Constants;
import frc.robot.subsystems.ControlPanelController;

public class AutoPanel extends CommandBase {

	private ControlPanelController controlPanelController;
	private BooleanSupplier activate;
	private Color previous;
	private int index;


	public AutoPanel(ControlPanelController controlPanelController, BooleanSupplier activate, int index) {
		this.activate = activate;
		this.index = index;
		this.controlPanelController = controlPanelController;   
		addRequirements(controlPanelController);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// Set Inital Color
		this.previous = controlPanelController.getColor();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		//turns the control panel based on the current situation
		if(activate.getAsBoolean()){
			if(controlPanelController.isPannelComplete() == false){
				// Start turning the Motor to turn the control pannel
				controlPanelController.turnSpeed(0.5);

				if(controlPanelController.getRotCompletion() == false){
					Rotate();
				}
				else{
					// Rotations Complete
					if(controlPanelController.getColor() == controlPanelController.GameColor()){
						controlPanelController.panelCompleted(true);
					}
				}  
			}
			else{
				controlPanelController.turnSpeed(0);
			}
		}
	}

	public void Rotate(){

		// Update changed color
		if(controlPanelController.getColor() != previous){
			index += 1;
			previous = controlPanelController.getColor();
		}

		// Completed rotations
		if(index >= Constants.ControlPanelConstants.targetRotations){
			controlPanelController.rotationsCompleted(true);
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
