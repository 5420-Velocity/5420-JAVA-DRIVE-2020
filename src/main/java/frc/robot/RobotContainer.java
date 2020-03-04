/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.ButtonMapConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.AutoPanel;
import frc.robot.commands.JoystickDrive;
import edu.wpi.first.wpilibj.Compressor;
import frc.robot.commands.Shoot;
import frc.robot.commands.liftControl;
import frc.robot.subsystems.ChuteSubsystem;
import frc.robot.subsystems.ControlPanelController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LiftSubsystem;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

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


	private final NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();

	private final DriveTrain driveTrain = new DriveTrain();
	private final ControlPanelController controlPanelController = new ControlPanelController();

	private static int index;

	public Compressor compressor = new Compressor(0);

	private final AutoPanel autoPanel = new AutoPanel(controlPanelController, index, 
		() -> operatorJoystick.getRawButton(Constants.ButtonMapConstants.Yellow_Button_ID),
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

	private final liftControl liftCommand = new liftControl(lift, intake,
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

	/**
	 * The container for the robot.  Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		// Call of the configuration helper functions.
		configurePIDControllers();
		configureNetworkTableBindings();
		configureButtonBindings();
		configureDefaultCommands();
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
			.whenPressed(() -> this.shooter.setSpeed(-0.6, 0.7))
			.whenReleased(() -> this.shooter.setSpeed(0,0));

		new JoystickButton(this.operatorJoystick, Constants.ButtonMapConstants.Left_Bumper)
			.whenPressed(() -> this.chute.setLeft(-0.35))
			.whenReleased(() -> this.chute.setLeft(0));

		new JoystickButton(this.operatorJoystick, Constants.ButtonMapConstants.Right_Bumper)
			.whenPressed(() -> this.chute.setRight(0.35))
			.whenReleased(() -> this.chute.setRight(0));
		

		/**
		 * Setup the button event for the Intake on the Operator Controller
		 */
		new JoystickButton(this.operatorJoystick, ButtonMapConstants.Green_Button_ID)
			// Go Down on Button Press
			.whenPressed(() -> this.intake.intakeMove(0.8))
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
