package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.NewShooterSubsystem.coverState;
import frc.robot.utils.JoystickDPad;
import frc.robot.utils.DPad.Position;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	public enum Side {
		Left,
		Right;
	}

	// The robot's subsystems and commands are defined here..
	private final Joystick driverJoystick = new Joystick(ControllerConstants.JOYSTICK_USB_DRIVER);
	private final Joystick operatorJoystick = new Joystick(ControllerConstants.JOYSTICK_USB_OPERATOR);
	public final DriveTrain driveTrain = new DriveTrain();
	private final PIDController drivePidController = new PIDController(
		DriveTrainConstants.LongEncoderP,
		DriveTrainConstants.LongEncoderI,
		DriveTrainConstants.LongEncoderD);
	private final ControlPanelController controlPanelController = new ControlPanelController();
	public Compressor compressor = new Compressor(0);
	private final SendableChooser<Command> autoChooser = new SendableChooser<>();

	// private final JoystickDrive joystickDrive = new JoystickDrive(driveTrain,
	// 	// Arcade
	// 	() -> driverJoystick.getRawAxis(5),
	// 	() -> driverJoystick.getRawAxis(4),
	// 	driverJoystick
	// );

	// // USB PS2 Controllers
	// private final JoystickDrive joystickDrive = new JoystickDriveTankdrive(driveTrain,
	// 	// Split Arcade
	// 	() -> driverJoystick.getRawButton(1),
	// 	() -> driverJoystick.getRawAxis(1),
	// 	() -> operatorJoystick.getRawAxis(1),
	// 	driverJoystick,
	// 	operatorJoystick
	// );

	// private final JoystickDrive joystickDrive = new JoystickDrive(driveTrain,
	// 	// Arcade w/ Expo
	// 	() -> Math.pow(driverJoystick.getRawAxis(5), 3) + 2 * driverJoystick.getRawAxis(5),
	// 	() -> Math.pow(driverJoystick.getRawAxis(4), 3) + 2 * driverJoystick.getRawAxis(4),
	// 	driverJoystick
	// );

	// private final JoystickDrive joystickDrive = new JoystickDriveArcadeSplit(driveTrain,
	// 	// Split Arcade
	// 	() -> this.applyCurve(driverJoystick.getRawAxis(1)),
	// 	() -> this.applyCurve(driverJoystick.getRawAxis(4)),
	// 	driverJoystick,
	// 	() -> driverJoystick.getRawAxis(2),
	// 	() -> driverJoystick.getRawAxis(3)
	// );

	private final JoystickDrive joystickDrive = new JoystickDriveLean(driveTrain,
		// Split Arcade
		() -> this.applyCurve(driverJoystick.getRawAxis(1)),
		() -> this.applyCurve(driverJoystick.getRawAxis(4)),
		driverJoystick,
		() -> driverJoystick.getRawAxis(2),
		() -> driverJoystick.getRawAxis(3)
	);

	// private final JoystickDrive joystickDrive = new JoystickDriveTankdrive(driveTrain,
	// 	// Split Arcade
	// 	() -> driverJoystick.getRawAxis(5),
	// 	() -> driverJoystick.getRawAxis(1),
	// 	driverJoystick
	// );

	// private final JoystickDrive joystickDrive = new JoystickDriveTankdriveLocked(driveTrain,
	// 	// Split Arcade
	// 	() -> driverJoystick.getRawAxis(1),
	// 	() -> driverJoystick.getRawAxis(4),
	// 	driverJoystick
	// );

	/**
	 * Setup Intake Subsystem and the Extra Commands to Control It
	 */
	public final Intake intake = new Intake();
	public final CharlesSubsystem charles = new CharlesSubsystem();
	private final NewShooterSubsystem newShooter = new NewShooterSubsystem();
	private final LiftSubsystem lift = new LiftSubsystem();
	private final ChuteSubsystem chute = new ChuteSubsystem();
	public AtomicBoolean forceHighValue = new AtomicBoolean(true);



	// private final Shoot shoot = new Shoot(shooter,
	// 	() -> operatorJoystick.getRawButton(Constants.ControllerMapConstants.Blue_Button_ID),
	// 	() -> operatorJoystick.getRawButton(5),
	// 	() -> operatorJoystick.getRawButton(4)
	// );

	// private final NewShoot newShoot = new NewShoot(charles, newShooter,
	// 	() -> operatorJoystick.getRawButton(Constants.ControllerMapConstants.Blue_Button_ID)
	// );

	// Encoder PID
	private final PIDController pidController = new PIDController(
		IntakeConstants.Proportional,
		IntakeConstants.Integral,
		IntakeConstants.Derivative);

	public final Limelight limeLight = new Limelight("one", ShooterConstants.knownArea, ShooterConstants.knownDistance);

	/**
	 * The container for the robot.  Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Initializing camera server for usb cameras attached to the rio
		// CameraServer.getInstance().startAutomaticCapture();

		// Initialize the Limelight to turn it off.
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

		PIDController autoTurnPIDController = new PIDController(
			DriveTrainConstants.TurnP,
			DriveTrainConstants.TurnI,
			DriveTrainConstants.TurnD
		);

		AtomicReference<Double> turnOutput = new AtomicReference<Double>(0.0);

		this.autoChooser.addOption("BallsMiddle", new SequentialCommandGroup(
			new ShuffleboardDelay("Auto Delay", 0),
			//new DriveWithEncoder(this.driveTrain, 0, true, 0.7),
			// new FunctionCommand(() -> {
			// 	this.limeLight.setLedMode(0);
			// }),
			// new FinishablePIDCommand(
			// 		autoTurnPIDController,
			// 			limeLight::getTX,
			// 			0.0,
			// 			output -> {
			// 						double turnSpeed = output;
			// 						System.out.println(limeLight.getTX());
			// 						// Set a max speed
			// 						turnSpeed = MathUtil.clamp(turnSpeed, -0.8, 0.8);
		
			// 						if (!this.limeLight.hasTarget()) {
			// 							// No Target
			// 							turnSpeed = 0;
			// 						}
		
			// 						driveTrain.arcadeDrive(0, turnSpeed);
			// 						//System.out.println(turnSpeed);
			// 			 	},
			// 			FinishablePIDCommand.ConsumeValueType.Offset,
			// 			offset -> {
			// 				if(!this.limeLight.hasTarget()) {
			// 					return false;
			// 				}

			// 				// Check LL to see if the values are "stable" or "within range" of our goal.
			// 				// Return true will kill this command.
			// 				if (Math.abs(offset) < 1) {
			// 					SmartDashboard.putBoolean("Turning Complete", true);
			// 					return true;
			// 				}
			// 				SmartDashboard.putBoolean("Turning Complete", false);
			// 				//System.out.println(offset);
			// 				return false;
			// 			},
			// 			driveTrain
			// 		),
				new AutoShoot(newShooter, chute, 0.45),
				new DriveWithEncoder(this.driveTrain, 60, true, 0.45),
				new TurnWithTime(this.driveTrain, 1000, Side.Left),
				new DriveWithEncoder(this.driveTrain, 25, true, 0.45),

				new PixySearch(this.intake, this.driveTrain, Side.Right),
				new PixyTurn(this.intake, this.driveTrain),
				new BackgroundCommandGroup(
					new PixyDrive(this.intake, this.driveTrain),
					new IntakePIDCommand(this.pidController, this.intake)
				),
				new PixySearch(this.intake, this.driveTrain, Side.Right),
				new PixyTurn(this.intake, this.driveTrain),
				new BackgroundCommandGroup(
					new PixyDrive(this.intake, this.driveTrain),
					new IntakePIDCommand(this.pidController, this.intake)
				),
				new PixySearch(this.intake, this.driveTrain),
				new PixyTurn(this.intake, this.driveTrain),
				new BackgroundCommandGroup(
					new PixyDrive(this.intake, this.driveTrain),
					new IntakePIDCommand(this.pidController, this.intake)
				)
			
		));

		this.autoChooser.addOption("BallsRight", new SequentialCommandGroup(
			//new DriveWithEncoder(this.driveTrain, 0, true, 0.7),
			new ShuffleboardDelay("Auto Delay", 0),
			new FunctionCommand(() -> {
				this.limeLight.setLedMode(0);
			}),
			// new FinishablePIDCommand(
			// 		autoTurnPIDController,
			// 			limeLight::getTX,
			// 			0.0,
			// 			output -> {
			// 						double turnSpeed = output;
			// 						System.out.println(limeLight.getTX());
			// 						// Set a max speed
			// 						turnSpeed = MathUtil.clamp(turnSpeed, -0.8, 0.8);
		
			// 						if (!this.limeLight.hasTarget()) {
			// 							// No Target
			// 							turnSpeed = 0;
			// 						}
		
			// 						driveTrain.arcadeDrive(0, turnSpeed);
			// 						//System.out.println(turnSpeed);
			// 			 	},
			// 			FinishablePIDCommand.ConsumeValueType.Offset,
			// 			offset -> {
			// 				if(!this.limeLight.hasTarget()) {
			// 					return false;
			// 				}

			// 				// Check LL to see if the values are "stable" or "within range" of our goal.
			// 				// Return true will kill this command.
			// 				if (Math.abs(offset) < 1) {
			// 					SmartDashboard.putBoolean("Turning Complete", true);
			// 					return true;
			// 				}
			// 				SmartDashboard.putBoolean("Turning Complete", false);
			// 				//System.out.println(offset);
			// 				return false;
			// 			},
			// 			driveTrain
			// 		),
					new AutoShoot(newShooter, chute, 0.45),
					new DriveWithEncoder(this.driveTrain, 60, true, 0.45),
					new TurnWithTime(this.driveTrain, 300, Side.Right),
					new FunctionCommand(() -> {
						this.limeLight.setLedMode(1);
					}),

				new PixySearch(this.intake, this.driveTrain, Side.Right),
				new PixyTurn(this.intake, this.driveTrain),
				new BackgroundCommandGroup(
					new PixyDrive(this.intake, this.driveTrain),
					new IntakePIDCommand(this.pidController, this.intake)
				),
				new PixySearch(this.intake, this.driveTrain, Side.Right),
				new PixyTurn(this.intake, this.driveTrain),
				new BackgroundCommandGroup(
					new PixyDrive(this.intake, this.driveTrain),
					new IntakePIDCommand(this.pidController, this.intake)
				),
				new PixySearch(this.intake, this.driveTrain),
				new PixyTurn(this.intake, this.driveTrain),
				new BackgroundCommandGroup(
					new PixyDrive(this.intake, this.driveTrain),
					new IntakePIDCommand(this.pidController, this.intake)
				)

					//  new RepeatedCommand(3, new SequentialCommandGroup(
					// 	new RunnableCommand(() -> System.out.println("started")),
					// 	new PixySearch(this.intake, this.driveTrain, Side.Right),
					// 	new PixyTurn(this.intake, this.driveTrain),
					// 	new BackgroundCommandGroup(
					// 		new PixyDrive(this.intake, this.driveTrain),
					// 		new IntakePIDCommand(this.pidController, this.intake)
					// 	)))
					
		));

		this.autoChooser.addOption("Balls", new SequentialCommandGroup(
			new PixySearch(this.intake, this.driveTrain),
			new PixyTurn(this.intake, this.driveTrain),
			new BackgroundCommandGroup(
				new PixyDrive(this.intake, this.driveTrain),
				new IntakePIDCommand(this.pidController, this.intake)
			),
			new PixySearch(this.intake, this.driveTrain),
			new PixyTurn(this.intake, this.driveTrain),
			new BackgroundCommandGroup(
				new PixyDrive(this.intake, this.driveTrain),
				new IntakePIDCommand(this.pidController, this.intake)
			),
			new PixySearch(this.intake, this.driveTrain),
			new PixyTurn(this.intake, this.driveTrain),
			new BackgroundCommandGroup(
				new PixyDrive(this.intake, this.driveTrain),
				new IntakePIDCommand(this.pidController, this.intake)
			)
		));

		this.autoChooser.addOption("BallsLeft", new SequentialCommandGroup(
			//new DriveWithEncoder(this.driveTrain, 0, true, 0.7),
				new ShuffleboardDelay("Auto Delay", 0),
				new AutoShoot(newShooter, chute, 0.45),
				new TurnWithTime(this.driveTrain, 2300, Side.Left),
				new DriveWithEncoder(this.driveTrain, 100, false, 0.5),
				new FunctionCommand(() -> {
					this.driveTrain.setBrakeMode(NeutralMode.Coast);
				})

		));

		this.autoChooser.addOption("Balls", new SequentialCommandGroup(
			new PixySearch(this.intake, this.driveTrain),
			new PixyTurn(this.intake, this.driveTrain),
			new BackgroundCommandGroup(
				new PixyDrive(this.intake, this.driveTrain),
				new IntakePIDCommand(this.pidController, this.intake)
			),
			new PixySearch(this.intake, this.driveTrain),
			new PixyTurn(this.intake, this.driveTrain),
			new BackgroundCommandGroup(
				new PixyDrive(this.intake, this.driveTrain),
				new IntakePIDCommand(this.pidController, this.intake)
			),
			new PixySearch(this.intake, this.driveTrain),
			new PixyTurn(this.intake, this.driveTrain),
			new BackgroundCommandGroup(
				new PixyDrive(this.intake, this.driveTrain),
				new IntakePIDCommand(this.pidController, this.intake)
			)
		));


		//positive power is intake forward
		this.autoChooser.setDefaultOption("Do Nothing", new DoNothingAutoCommand());
		this.autoChooser.addOption("Barrel Racing", new SequentialCommandGroup(
			//Drive 90 inches
			new DriveWithEncoder(this.driveTrain, 84, false, 0.9),

			// Lean 1 full revolution
			new LeanWithEncoder(this.driveTrain, 20, Side.Right, -0.4, 1.08),

			//Drive forward 135 inches
			new DriveWithEncoder(this.driveTrain, 76, false, 0.9),

			// Lean 1 full revolution
			new LeanWithEncoder(this.driveTrain, 20, Side.Left, -0.4, 0.82),

			//drive forward 30 inches
			new DriveWithEncoder(this.driveTrain, 80, false, 0.9),

			// Lean 1/2 revolution
			new LeanWithEncoder(this.driveTrain, 20, Side.Left, -0.4, 0.59),

			// Drive forward 240 inches
			new DriveWithEncoder(this.driveTrain, 210, false, 1)
		));

		this.autoChooser.addOption("Bounce Path", new SequentialCommandGroup(
			// Init distance, tune for first turn entry
			new DriveWithEncoder(this.driveTrain, 30, false, 0.6, 3),
			new LeanWithEncoder(this.driveTrain, 20, Side.Left, -0.3, 0.155),
			new DriveWithEncoder(this.driveTrain, 30, false, 0.6, 3),
			new LeanWithEncoder(this.driveTrain, 20, Side.Left, -0.3, 0.03),
			new DriveWithEncoder(this.driveTrain, 75, true, 0.6, 4),
			new LeanWithEncoder(this.driveTrain, 20, Side.Right, 0.3, 0.09),
			new DriveWithEncoder(this.driveTrain, 20, true, 0.6, 3),
			new LeanWithEncoder(this.driveTrain, 20, Side.Right, 0.3, 0.11),
			new DriveWithEncoder(this.driveTrain, 100, true, 0.6, 3),
			new DriveWithEncoder(this.driveTrain, 90, false, 0.6, 3),
			new LeanWithEncoder(this.driveTrain, 20, Side.Left, -0.3, 0.14),
			new DriveWithEncoder(this.driveTrain, 45, false, 0.6, 3),
			new LeanWithEncoder(this.driveTrain, 20, Side.Left, -0.3, 0.155),
			new DriveWithEncoder(this.driveTrain, 94, false, 0.6, 3),
			new DriveWithEncoder(this.driveTrain, 14, true, 0.6, 3),
			new LeanWithEncoder(this.driveTrain, 20, Side.Right, 0.3, 0.14),
			new DriveWithEncoder(this.driveTrain, 20, true, 0.6, 3)

			// new DriveWithEncoder(this.driveTrain, 30, true, 0.8),
			// new DriveWithEncoder(this.driveTrain, 28, false, 0.8),
			// new LeanWithEncoder(this.driveTrain, 20, Side.Left, -0.4, 0.25),
			// new DriveWithEncoder(this.driveTrain, 10, false, 0.8),
			// new LeanWithEncoder(this.driveTrain, 20, Side.Left, -0.4, 0.25),
			// new DriveWithEncoder(this.driveTrain, 28, false, 0.8),
			// new DriveWithEncoder(this.driveTrain, 5, true, 0.8),
			// new LeanWithEncoder(this.driveTrain, 20, Side.Left, -0.4, 0.25),
			// new DriveWithEncoder(this.driveTrain, 10, false, 0.8)
		));

		this.autoChooser.addOption("Slolom Path", new SequentialCommandGroup(
			// // Init distance, tune for first turn entry
			new DriveWithEncoder(this.driveTrain, 27, false, 0.6, 3),
			new LeanWithEncoder(this.driveTrain, 20, Side.Left, -0.3, 0.13),
			new DriveWithEncoder(this.driveTrain, 35, false, 0.6, 3),
			new LeanWithEncoder(this.driveTrain, 20, Side.Right, -0.3, 0.075),
			new DriveWithEncoder(this.driveTrain, 90, false, 0.9, 8),
			new LeanWithEncoder(this.driveTrain, 20, Side.Right, -0.3, 0.45),
			new DriveWithEncoder(this.driveTrain, 30, false, 0.6, 3),
			new LeanWithEncoder(this.driveTrain, 25, Side.Left, -0.4, 1),
			new DriveWithEncoder(this.driveTrain, 28, false, 0.6, 3),
			new LeanWithEncoder(this.driveTrain, 20, Side.Right, -0.3, 0.09),
			new DriveWithEncoder(this.driveTrain, 100, false, 0.9, 9),
			new LeanWithEncoder(this.driveTrain, 20, Side.Right, -0.3, 0.44),
			new DriveWithEncoder(this.driveTrain, 35, false, 0.8, 3),
			new LeanWithEncoder(this.driveTrain, 20, Side.Left, -0.3, 0.13),
			new DriveWithEncoder(this.driveTrain, 30, false, 0.6, 3)

		));

		// this.autoChooser.addOption("Barrel Racing", new PathWeaverAuto(this.driveTrain, "PathWeaver/Barrel Racing/Groups/AutoNav.json"));
		// this.autoChooser.addOption("Bounce Path", new PathWeaverAuto(this.driveTrain, "paths/YourPath.wpilib.json"));
		// this.autoChooser.addOption("Slolom Path", new PathWeaverAuto(this.driveTrain, "paths/YourPath.wpilib.json"));

		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	/**
	 * Configure PID Controllers that are stored in the current
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

	private double applyCurve(double value) {
		return value;
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
		 * Lambda functions only allow references not variables of direct types like doubles.
		 *
		 */
		AtomicReference<Double> turnOutput = new AtomicReference<Double>(0.0);

		SmartDashboard.setDefaultBoolean("Range Complete", false);
		SmartDashboard.setDefaultBoolean("Turing Complete", false);

		new JoystickButton(this.operatorJoystick, ControllerMapConstants.Red_Button_ID)
			.whenPressed(() -> this.newShooter.coverSet(coverState.Up))
			.whenReleased(() -> this.newShooter.coverSet(coverState.Down));

		new JoystickButton(this.operatorJoystick, ControllerMapConstants.Yellow_Button_ID)
			.whenPressed(() -> this.limeLight.setLedMode(0))
			.whenReleased(() -> this.limeLight.setLedMode(1));

		new JoystickButton(this.driverJoystick, ControllerMapConstants.Green_Button_ID)
			// Enable the Limelight LED
			.whenPressed(() -> this.limeLight.setLedMode(0))
			// Disable the Limelight LED
			.whenReleased(() -> this.limeLight.setLedMode(1))
			
			.whenHeld(
				new SequentialCommandGroup(
					new FinishablePIDCommand(
					turnPIDController,
						limeLight::getTX,
						0.0,
						output -> {
									double turnSpeed = output;
									// Set a max speed
									turnSpeed = MathUtil.clamp(turnSpeed, -0.8, 0.8);
		
									if (!this.limeLight.hasTarget()) {
										// No Target
										turnSpeed = 0;
									}
		
									driveTrain.arcadeDrive(0, turnSpeed);
						 	},
						FinishablePIDCommand.ConsumeValueType.Offset,
						offset -> {
							if(!this.limeLight.hasTarget()) {
								return false;
							}

							// Check LL to see if the values are "stable" or "within range" of our goal.
							// Return true will kill this command.
							if (Math.abs(offset) < 1) {
								SmartDashboard.putBoolean("Turning Complete", true);
								return true;
							}
							SmartDashboard.putBoolean("Turning Complete", false);
							//System.out.println(offset);
							return false;
						},
						driveTrain
					))
				);
				

			// .whenHeld(new SequentialCommandGroup(
			// 	new ParallelCommandGroup(
			// 		// Range
			// 		new FinishablePIDCommand(
			// 			rangePIDController,
			// 			this.limeLight::getDistance,
			// 			Constants.ShooterConstants.rangeGoal,
			// 			output -> {
			// 				double turnSpeed = turnOutput.get();
			// 				double outSpeed = output;

			// 				// Set a max speed
			// 				turnSpeed = MathUtil.clamp(turnSpeed, -0.8, 0.8);
			// 				outSpeed = MathUtil.clamp(outSpeed, -0.5, 0.5);

			// 				if (!this.limeLight.hasTarget()) {
			// 					// No Target
			// 					outSpeed = -0.35;
			// 					turnSpeed = 0;
			// 				}

			// 				driveTrain.arcadeDrive(outSpeed, turnSpeed);
			// 			},
			// 			FinishablePIDCommand.ConsumeValueType.Offset,
			// 			offset -> {
			// 				// Check LL to see if the values are "stable" or "within range" of our goal.
			// 				// Return true will kill this command.

			// 				if (Math.abs(offset) < 2) {
			// 					SmartDashboard.putBoolean("Range Complete", true);
			// 					return true;
			// 				}
			// 				SmartDashboard.putBoolean("Range Complete", false);
			// 				return false;
			// 			},
			// 			driveTrain
			// 		),
			// 		// Turning
			// 		new FinishablePIDCommand(
			// 			turnPIDController,
			// 			limeLight::getTX,
			// 			0.0,
			// 			turnOutput::set,
			// 			FinishablePIDCommand.ConsumeValueType.Offset,
			// 			offset -> {
			// 				// Check LL to see if the values are "stable" or "within range" of our goal.
			// 				// Return true will kill this command.
			// 				if (Math.abs(offset) < 1) {
			// 					SmartDashboard.putBoolean("Turning Complete", true);
			// 					return true;
			// 				}
			// 				SmartDashboard.putBoolean("Turning Complete", false);
			// 				return false;
			// 			},
			// 			driveTrain
			// 		)
			// 	)
			// //	new AutoShoot(this.newShooter, this.chute, shooterSpeed.get())
			// ));

		new JoystickButton(this.operatorJoystick, Constants.ControllerMapConstants.Joystick_Left_Button)
			.whenPressed(new SequentialCommandGroup(
				new FunctionCommand(() -> {
					this.driveTrain.setBrakeMode(NeutralMode.Coast);
				}),
				new PanelLiftDown(controlPanelController)
			));

		new JoystickButton(this.operatorJoystick, Constants.ControllerMapConstants.Joystick_Right_Button)
			.whenPressed(new SequentialCommandGroup(
				new FunctionCommand(() -> {
					this.driveTrain.setBrakeMode(NeutralMode.Brake);
				}),
				new PanelLiftUp(controlPanelController)
			));

		// new JoystickButton(this.operatorJoystick, Constants.ControllerMapConstants.Joystick_Left_Button)
		// 	.whenPressed(new PanelLiftDown(controlPanelController));

		// new JoystickButton(this.operatorJoystick, Constants.ControllerMapConstants.Joystick_Right_Button)
		// 	.whenPressed(new PanelLiftUp(controlPanelController));

		/**
		 * Setup Button Events for the Shooter on the Driver Controller
		 */
		new JoystickButton(this.driverJoystick, Constants.ControllerMapConstants.Right_Bumper)
			.whenPressed(() -> this.driveTrain.shift(true))
			.whenReleased(() -> this.driveTrain.shift(false));

		// Update the command settings to flip the controls
		new JoystickButton(this.driverJoystick, Constants.ControllerMapConstants.Red_Button_ID)
			.whenPressed(this.joystickDrive::toggleFlipped);

		new JoystickButton(this.driverJoystick, Constants.ControllerMapConstants.Yellow_Button_ID)
			.whenPressed(this.driveTrain::resetEncoders);

		/**
		 * Used to dynamically adjust the speed used for shooting.
		 */
		AtomicReference<Double> shooterSpeed = new AtomicReference<Double>(0.5);

		/**
		 * Setup Button Events for the Shooter on the Operator Controller
		 */
		new JoystickButton(this.operatorJoystick, Constants.ControllerMapConstants.Blue_Button_ID)
			.whileHeld(() -> this.newShooter.setSpeed(shooterSpeed.get(), 0))
			.whenReleased(() -> this.newShooter.setSpeed(0, 0));

		new JoystickDPad(this.operatorJoystick, Position.kUp)
			.whenPressed(() -> {
				double increaseBy = 0.01;
				double newSpeed = shooterSpeed.get() + increaseBy;
				shooterSpeed.set(newSpeed);
			});

		new JoystickDPad(this.operatorJoystick, Position.kDown)
			.whenPressed(() -> {
				double decreaseBy = -0.01;
				double newSpeed = shooterSpeed.get() + decreaseBy;
				shooterSpeed.set(newSpeed);
			});

		// new JoystickButton(this.operatorJoystick, Constants.ControllerMapConstants.Left_Bumper)
		// 	.whenPressed(() -> this.chute.setLeft(-0.6))
		// 	.whenReleased(() -> this.chute.setLeft(0));

		new JoystickButton(this.operatorJoystick, Constants.ControllerMapConstants.Left_Bumper)
			.whileActiveOnce(new AutoShoot(this.newShooter, this.chute, shooterSpeed));

		new JoystickButton(this.operatorJoystick, Constants.ControllerMapConstants.Right_Bumper)
			.whenPressed(() -> this.chute.setRight(0.6))
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
					this.intake::getEncoderFromLowValue,
					0.0,
					this.intake::armSpeed,
					this.intake
				)
			)
			// Turn off Motor
			.whenReleased(() -> this.intake.intakeMove(0));

			new JoystickButton(this.driverJoystick, ControllerMapConstants.Blue_Button_ID)
				.whenPressed(() -> {
					this.forceHighValue.set(!forceHighValue.get());
				});


		// String trajectoryJSON = "paths/YourPath.wpilib.json";
		// Trajectory trajectory = new Trajectory();
		// try {
		// 	Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
		// 	trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		// } catch (IOException ex) {
		// 	DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
		// }

	}

	/**
	 * Configure Default Commands for the Subsystems
	 */
	private void configureDefaultCommands() {
		CommandScheduler scheduler = CommandScheduler.getInstance();

		scheduler.setDefaultCommand(this.driveTrain, this.joystickDrive);

		scheduler.setDefaultCommand(this.lift, new LiftControl(lift, intake,
			() -> operatorJoystick.getRawAxis(Constants.ControllerMapConstants.Right_Trigger),
			() -> operatorJoystick.getRawAxis(Constants.ControllerMapConstants.Left_Trigger)
		));

		// Go Up By Default
		scheduler.setDefaultCommand(this.intake, new PIDCommand(
			this.pidController,
			() -> {
				if (this.forceHighValue.get()) {
					return this.intake.getEncoderFromHighValue();
				}
				return this.intake.getEncoderFromLowValue();
			},
			0.0,
			intake::armSpeed,
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
