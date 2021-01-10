package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.concurrent.atomic.AtomicReference;

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
	private final SendableChooser<Command> autoChooser = new SendableChooser<>();

	private final JoystickDrive joystickDrive = new JoystickDrive(driveTrain,
		() -> driverJoystick.getRawAxis(1),
		() -> driverJoystick.getRawAxis(4),
		driverJoystick
	);

	/**
	 * Setup Intake Subsystm and the Extra Commands to Contorl It
	 */
	public final Intake intake = new Intake();
	public final CharlesSubsystem charles = new CharlesSubsystem();
	private final ShooterSubsystem shooter = new ShooterSubsystem();
	private final NewShooterSubsystem newShooter = new NewShooterSubsystem();
	private final LiftSubsystem lift = new LiftSubsystem();
	private final ChuteSubsystem chute = new ChuteSubsystem();

	// private final Shoot shoot = new Shoot(shooter,
	// 	() -> operatorJoystick.getRawButton(Constants.ControllerMapConstants.Blue_Button_ID),
	// 	() -> operatorJoystick.getRawButton(5),
	// 	() -> operatorJoystick.getRawButton(4)
	// );

	private final NewShoot newShoot = new NewShoot(charles, newShooter,
		() -> operatorJoystick.getRawButton(Constants.ControllerMapConstants.Blue_Button_ID)
	);

	// Encoder PID
	private PIDController pidController = new PIDController(
		IntakeConstants.Proportional,
		IntakeConstants.Integral,
		IntakeConstants.Derivative);

	public final Limelight limeLight = new Limelight("one", ShooterConstants.knownArea, ShooterConstants.knownDistance);

	/**
	 * The container for the robot.  Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Initializing camera server for usb cameras atached to the rio
		// CameraServer.getInstance().startAutomaticCapture();

		// Initalize the Limelight to turn it off.
		this.limeLight.setLedMode(1);

		// Call of the configuration helper functions.
		configurePIDControllers();
		configureNetworkTableBindings();
		configureButtonBindings();
		configureDefaultCommands();
		configureAutoChooser();
	}

	/**
	 * Configure the auto commands
	 */
	private void configureAutoChooser() {

		this.autoChooser.setDefaultOption("Do Nothing", new DoNothingAutoCommand());
		this.autoChooser.addOption("Color Wheel Test", new SequentialCommandGroup(
			new PanelLiftUp(this.controlPanelController),
			new AutoPanelColorTickTurn(this.controlPanelController, new AtomicReference<Boolean>())
		));

		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	/**
	 * Configure PID Controlelrs that are stored in the current
	 * RobotContainer instance.
	 */
	private void configurePIDControllers() {

		this.pidController.setTolerance(0.05);
		this.pidController.setIntegratorRange(-0.7, 0.7);

	}

	/**
	 * Configure network table entries with the default values.
	 */
	private void configureNetworkTableBindings() {

	}

	/**
	 * Define button to command mappings.
	 */
	private void configureButtonBindings() {

		// Limelight ctrl
		PIDController rangePIDController = new PIDController(
			DriveTrainConstants.RangeP,
			DriveTrainConstants.RangeI,
			DriveTrainConstants.RangeD);
		PIDController turnPIDController = new PIDController(
			DriveTrainConstants.TurnP,
			DriveTrainConstants.TurnI,
			DriveTrainConstants.TurnD
		);

		/**
		 * Using AtomicReference we are using two PID Commands at the same time
		 *  allowing one PID Command to actually set the motor value and use
		 * the AtomicReference<Double> to hold the turn value.
		 * Lambda functions only allow refrences not variables of direct types like doubles.
		 *
		 */
		AtomicReference<Double> turnOutput = new AtomicReference<Double>();

		new JoystickButton(this.driverJoystick, ControllerMapConstants.Green_Button_ID)
			// Enable the Limelight LED
			.whenPressed(() -> this.limeLight.setLedMode(0))
			// Disable the Limelight LED
			.whenReleased(() -> this.limeLight.setLedMode(1))
			.whenHeld(new ParallelCommandGroup(
				// Turning
				new PIDCommand(
					turnPIDController,
					limeLight::getTX,
					0.0,
					output -> turnOutput.set(output),
					driveTrain
				),
				// Range
				new PIDCommand(
					rangePIDController,
					() -> {
						// If no target is found, Offset is Zero
						if (this.limeLight.hasTarget() == false) return 0.0;
						return this.limeLight.getDistance() - Constants.ShooterConstants.rangeGoal;
					},
					0.0,
					output -> {
						System.out.println("Motor Value: Y" + turnOutput.get() + " X" + output);
						driveTrain.arcadeDrive(output, turnOutput.get());
					},
					driveTrain
				)
			));

		new JoystickButton(this.operatorJoystick, Constants.ControllerMapConstants.Joystick_Left_Button)
			.whenPressed(new PanelLiftDown(controlPanelController));

		new JoystickButton(this.operatorJoystick, Constants.ControllerMapConstants.Joystick_Right_Button)
			.whenPressed(new PanelLiftUp(controlPanelController));

		/**
		 * Setup Button Events for the Shooter on the Driver Controller
		 */
		new JoystickButton(this.driverJoystick, Constants.ControllerMapConstants.Right_Bumper)
			.whenPressed(() -> this.driveTrain.shift(true))
			.whenReleased(() -> this.driveTrain.shift(false));

		// Update the command settings to flip the controls
		new JoystickButton(this.driverJoystick, Constants.ControllerMapConstants.Red_Button_ID)
			.whenPressed(() -> this.joystickDrive.toggleFlipped());

		/**
		 * Setup Button Events for the Shooter on the Operator Controller
		 */
		new JoystickButton(this.operatorJoystick, Constants.ControllerMapConstants.Blue_Button_ID)
			.whenPressed(() -> this.newShooter.setSpeed(-0.65, -0.67, 0))
			.whenReleased(() -> this.newShooter.setSpeed(0, 0, 0));

		new JoystickButton(this.operatorJoystick, Constants.ControllerMapConstants.Left_Bumper)
			.whenPressed(() -> this.chute.setLeft(-0.7))
			.whenReleased(() -> this.chute.setLeft(0));

		new JoystickButton(this.operatorJoystick, Constants.ControllerMapConstants.Right_Bumper)
			.whenPressed(() -> this.chute.setRight(0.7))
			.whenReleased(() -> this.chute.setRight(0));


		/**
		 * Setup the button event for the Intake on the Operator Controller
		 */
		new JoystickButton(this.operatorJoystick, ControllerMapConstants.Green_Button_ID)
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

		new JoystickButton(this.operatorJoystick, ControllerMapConstants.Left_Bumper);
		//loading


	}

	/**
	 * Configure Default Commands for the Subsystems
	 */
	private void configureDefaultCommands() {
		CommandScheduler scheduler = CommandScheduler.getInstance();

		scheduler.setDefaultCommand(this.driveTrain, this.joystickDrive);

		scheduler.setDefaultCommand(this.lift, new LiftControl(lift, intake,
			() -> driverJoystick.getRawAxis(Constants.ControllerMapConstants.Right_Trigger),
			() -> driverJoystick.getRawAxis(Constants.ControllerMapConstants.Left_Trigger)
		));

		// Go Up By Default
		scheduler.setDefaultCommand(this.intake, new PIDCommand(
			this.pidController,
			() -> this.intake.getEncoderFromHighValue(),
			0.0,
			output -> intake.armRun(output),
			this.intake
		));

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return this.autoChooser.getSelected();
	}

}
