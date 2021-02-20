/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CharlesSubsystem;
import frc.robot.subsystems.NewShooterSubsystem;

import java.util.function.BooleanSupplier;

public class NewShoot extends CommandBase {
	/**
	 * Creates a new Shoot.
	 */
	private final NewShooterSubsystem shooter;
	private final CharlesSubsystem charles;
	private final BooleanSupplier startShoot;

	public NewShoot(CharlesSubsystem charles, NewShooterSubsystem shooter, BooleanSupplier startShoot) {
		this.shooter = shooter;
		this.charles = charles;
		this.startShoot = startShoot;
		addRequirements(shooter, charles);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// Shooting
		if (startShoot.getAsBoolean()) {
			shooter.setSpeed(-0.55, 0);
			this.charles.nextFilledLocation();
		}
		else {
			shooter.setSpeed(0, 0);
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
