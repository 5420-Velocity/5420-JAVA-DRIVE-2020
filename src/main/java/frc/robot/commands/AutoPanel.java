/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.ControlPanelController;

public class AutoPanel extends CommandBase {

	private ControlPanelController controlPanelController;
	private DoubleSupplier liftInput;
	private Color previous;
	private BooleanSupplier activate;
	private Boolean runRotate = false;
	private int index;


	public AutoPanel(ControlPanelController controlPanelController, int Index, BooleanSupplier activate, DoubleSupplier liftInput) {
		this.activate = activate;
		this.liftInput = liftInput;
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
		// Lift Control
		if(controlPanelController.getUpper() == false && liftInput.getAsDouble() > 0) {
			controlPanelController.liftSpeed(0);
		}
		else if(controlPanelController.getlower() == false && liftInput.getAsDouble() < 0) {
			controlPanelController.liftSpeed(0);
		}
		else {
			controlPanelController.liftSpeed(liftInput.getAsDouble());
		}

		// Turning control
		if(activate.getAsBoolean()){
			this.runRotate = true;
		}
		else{
			this.runRotate = false;
		}

		if(controlPanelController.isPannelComplete() == false && this.runRotate == true){
			// Start turning the Motor to turn the control pannel
			controlPanelController.turnSpeed(0.5);

			if(controlPanelController.getRotCompletion() == false){
				this.rotate();
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

	public void rotate(){

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
