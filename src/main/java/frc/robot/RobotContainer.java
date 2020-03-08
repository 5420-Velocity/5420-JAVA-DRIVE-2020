/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.ButtonMapConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutoPanelColorTickTurn;
import frc.robot.commands.AutoPanelDefaultCommand;
import frc.robot.commands.JoystickDrive;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import frc.robot.commands.Shoot;
import frc.robot.commands.LiftControl;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.ChuteSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ControlPanelController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LiftSubsystem;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here..
	private final Joystick driverJoystick = new Joystick(ControllerConstants.JOYSTICK_USB_DRIVER);
	private final Joystick operatorJoystick = new Joystick(ControllerConstants.JOYSTICK_USB_OPERATOR);

	private final DriveTrain driveTrain = new DriveTrain();
	private final ControlPanelController controlPanelController = new ControlPanelController();

	public Compressor compressor = new Compressor(0);

	private final AutoPanelDefaultCommand autoPanel = new AutoPanelDefaultCommand(
		controlPanelController,
		() -> -operatorJoystick.getRawAxis(Constants.ButtonMapConstants.JOYSTICK_LEFT_Y_AXIS)
	);

	private final JoystickDrive joystickDrive = new JoystickDrive(driveTrain, 
		() -> driverJoystick.getRawAxis(1),
		() -> driverJoystick.getRawAxis(4),
		driverJoystick
	);

	/**
	 * Setup Intake Subsystm and the Extra Commands to Contorl It
	 */
	final Intake intake = new Intake();

	private final ShooterSubsystem shooter = new ShooterSubsystem();

	private final LiftSubsystem lift = new LiftSubsystem();

	private final LiftControl liftCommand = new LiftControl(lift, intake,
		() -> driverJoystick.getRawAxis(Constants.ButtonMapConstants.Right_Trigger),
		() -> driverJoystick.getRawAxis(Constants.ButtonMapConstants.Left_Trigger)
	);

	private final ChuteSubsystem chute = new ChuteSubsystem();

	private final Shoot shoot = new Shoot(shooter,
		() -> operatorJoystick.getRawButton(Constants.ButtonMapConstants.Blue_Button_ID),
		() -> operatorJoystick.getRawButton(5),
		() -> operatorJoystick.getRawButton(4)
	);

	// Encoder PID
	private PIDController pidController = new PIDController(
		IntakeConstants.Proportional, 
		IntakeConstants.Integral, 
		IntakeConstants.Derivative);

	private final Limelight limeLight = new Limelight("one", ShooterConstants.knownArea, ShooterConstants.knownDistance);

	/**
	 * The container for the robot.  Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Initializing camera server for usb cameras atached to the rio
		// CameraServer.getInstance().startAutomaticCapture();

		// Call of the configuration helper functions.
		configurePIDControllers();
		configureNetworkTableBindings();
		configureButtonBindings();
		configureDefaultCommands();
		configureAuto();
	}
	/**
	 * Configure the auto commands
	 */

	private void configureAuto(){
		SequentialCommandGroup shootingAuto = new SequentialCommandGroup(
		
		);
	}

	/**
	 * Configure PID Controlelrs that are stored in the current
	 *  RobotContainer instance.
	 * 
	 */
	private void configurePIDControllers() {
		
		this.pidController.setTolerance(0.05);
		this.pidController.setIntegratorRange(-0.7, 0.7);

	}

	/**
	 * Configure network table entries with the default values.
	 * 
	 */
	private void configureNetworkTableBindings() {

	}

	/**
	 * Define button to command mappings.
	 * 
	 */
	private void configureButtonBindings() {

		// Limelight ctrl
		PIDController drivePIDController = new PIDController(
          DriveTrainConstants.DriveP, 
          DriveTrainConstants.DriveI, 
          DriveTrainConstants.DriveD);
		/*new JoystickButton(this.driverJoystick, ButtonMapConstants.Green_Button_ID)
			// Turning
			.whenHeld(new PIDCommand(
           		drivePIDController,
           		limeLight::getTX, 
           		0.0, 
           		output -> driveTrain.arcadeDrive(0.0, -output), 
				driveTrain))
		    // Range
		    .whenHeld(new PIDCommand(
				drivePIDController,
				limeLight::getDistanceError, 
				0.0, 
				output -> driveTrain.arcadeDrive(output, 0.0), 
				driveTrain));*/

		/**
		 * Setup Button Events for the Shooter on the Driver Controller
		 */
		new JoystickButton(this.driverJoystick, Constants.ButtonMapConstants.Right_Bumper)
			.whenPressed(() -> this.driveTrain.shift(true))
			.whenReleased(() -> this.driveTrain.shift(false));

		/**
		 * Setup Button Events for the Shooter on the Operator Controller
		 */
		new JoystickButton(this.operatorJoystick, Constants.ButtonMapConstants.Blue_Button_ID)
			.whenPressed(() -> this.shooter.setSpeed(-0.7, -0.90))
			.whenReleased(() -> this.shooter.setSpeed(0,0));

		new JoystickButton(this.operatorJoystick, Constants.ButtonMapConstants.Left_Bumper)
			.whenPressed(() -> this.chute.setLeft(-0.25))
			.whenReleased(() -> this.chute.setLeft(0));

		new JoystickButton(this.operatorJoystick, Constants.ButtonMapConstants.Right_Bumper)
			.whenPressed(() -> this.chute.setRight(0.25))
			.whenReleased(() -> this.chute.setRight(0));
		

		/**
		 * Setup the button event for the Intake on the Operator Controller
		 */
		new JoystickButton(this.operatorJoystick, ButtonMapConstants.Green_Button_ID)
			// Go Down on Button Press
			.whenPressed(() -> this.intake.intakeMove(-1.0))
			.whenHeld(
				new PIDCommand(
					this.pidController,
					() -> this.intake.getEncoderFromLowValue(),
					0.0,
					output -> this.intake.armRun(output),
					this.intake
				)
			)
			// Turn off Motor
			.whenReleased(() -> this.intake.intakeMove(0));

		/**
		 * Setup the button event for the Control Panel Turning with
		 *  the auto turn using the color sensor as a tick sensor.
		 */
		new JoystickButton(this.operatorJoystick, ButtonMapConstants.Yellow_Button_ID)
			.whenHeld(new AutoPanelColorTickTurn(controlPanelController));

	}

	/**
	 * Configure Default Commands for the Subsystems
	 * 
	 */
	private void configureDefaultCommands() {
		CommandScheduler scheduler = CommandScheduler.getInstance();

		scheduler.setDefaultCommand(this.controlPanelController, this.autoPanel);

		scheduler.setDefaultCommand(this.driveTrain, this.joystickDrive);

		scheduler.setDefaultCommand(this.lift, this.liftCommand);

		// Go Up By Default
		scheduler.setDefaultCommand(this.intake, new PIDCommand(
			this.pidController,
			() -> this.intake.getEncoderFromHighValue(),
			0.0,
			output -> intake.armRun(output),
			this.intake
		));
		
	}

}
