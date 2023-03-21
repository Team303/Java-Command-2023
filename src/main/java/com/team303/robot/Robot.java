package com.team303.robot;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.kauailabs.navx.frc.AHRS;
import com.team303.robot.RobotMap.IOConstants;
import com.team303.robot.RobotMap.LED;
import com.team303.robot.autonomous.Autonomous;
import com.team303.robot.autonomous.AutonomousProgram;
import com.team303.robot.commands.arm.DefaultIKControlCommand;
//import com.team303.robot.commands.arm.DefaultMove;
import com.team303.robot.commands.arm.Homing;
import com.team303.robot.commands.drive.AutolevelFeedforward;
import com.team303.robot.commands.drive.DefaultDrive;
import com.team303.robot.commands.drive.DriveWait;
import com.team303.robot.commands.drive.TurnToAngle;
import com.team303.robot.modules.Operator;
import com.team303.robot.modules.Photonvision;
import com.team303.robot.modules.Ultrasonic;
import com.team303.robot.subsystems.ArmSubsystem;
import com.team303.robot.subsystems.ArmTest;
import com.team303.robot.subsystems.ClawSubsystem;
import com.team303.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.BuildConstants;
import com.team303.robot.commands.claw.ClawOpwn;
import com.team303.robot.commands.claw.ClawClose;
import com.team303.robot.commands.claw.DefaultClaw;
import com.team303.robot.commands.arm.ReachAngles;

public class Robot extends LoggedRobot {

	/* RoboRio Sensors */
	private static final AHRS navX = new AHRS();
	/* Robot Subsystems */
	public static final Photonvision photonvision = null; //new Photonvision();
	public static final SwerveSubsystem swerve = new SwerveSubsystem();
	public static final ArmSubsystem arm = new ArmSubsystem();
	public static final ArmTest armtest = null; // new ArmTest();
	public static final Operator operator = null; //new Operator();
	public static final ClawSubsystem claw = new ClawSubsystem();
	public static final Ultrasonic ultrasonic = null; //new Ultrasonic(0, 4);

	/* Robot IO Controls */
	private static final Joystick leftJoystick = new Joystick(IOConstants.LEFT_JOYSTICK_ID);
	private static final Joystick rightJoystick = new Joystick(IOConstants.RIGHT_JOYSTICK_ID);
	private static final CommandXboxController operatorCommandXboxController = new CommandXboxController(
			IOConstants.OPERATOR_CONTROLLER);
	private static final XboxController operatorXboxController = new XboxController(IOConstants.OPERATOR_CONTROLLER);
	private static final CommandXboxController driverCommandXboxController = new CommandXboxController(
			IOConstants.DRIVER_CONTROLLER);
	private static final XboxController driverXboxController = new XboxController(IOConstants.DRIVER_CONTROLLER);

	public static SendableChooser<String> controllerChooser = new SendableChooser<>();

	/* Shufflebaord Tabs */
	public static final ShuffleboardTab AUTO_TAB = Shuffleboard.getTab("Autonomous");
	public static final ShuffleboardTab DATA_TAB = Shuffleboard.getTab("Data");
	public static final ShuffleboardTab CONTROLLER_TAB = Shuffleboard.getTab("Controller");

	public static final GenericEntry NAVX_Y_VELOCITY = DATA_TAB.add("Y velocity", 0).getEntry();
	public static final GenericEntry NAVX_ACCELERATION = DATA_TAB.add("acceleration", 0).getEntry();
	public static final GenericEntry NAVX_ANGLE = DATA_TAB.add("NavX Angle", 0).getEntry();
	public static final GenericEntry L_AXIS = CONTROLLER_TAB.add("l trigger", 0).getEntry();

	/* Shuffleboard Choosers */
	public static SendableChooser<Double> autoDelayChooser = new SendableChooser<>();
	public static HashMap<String, Command> eventMap = new HashMap<>();

	public static enum HeldObject {
		CUBE,
		CONE,
		NONE
	}

	public static HeldObject heldObject = HeldObject.NONE;

	/* Robot alliance color */
	public static Color allianceColor = DriverStation.getAlliance() == Alliance.Blue ? LED.RED : LED.BLUE;

	public static enum SubstationFiducialID {
		RED(5),
		BLUE(4);

		public final int fiducialId;

		private SubstationFiducialID(int fiducialId) {
			this.fiducialId = fiducialId;
		}
	}

	public static final int ALLIANCE_SUBSTATION_ID = DriverStation.getAlliance() == Alliance.Blue
			? SubstationFiducialID.RED.fiducialId
			: SubstationFiducialID.BLUE.fiducialId;
	private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();

	/* Getter Methods */

	public static AHRS getNavX() {
		return navX;
	}

	public static Joystick getRightJoyStick() {
		return rightJoystick;
	}

	public static Joystick getLeftJoyStick() {
		return leftJoystick;
	}

	public static XboxController getOperatorXbox() {
		return operatorXboxController;
	}

	public static CommandXboxController getOperatorCommandXbox() {
		return operatorCommandXboxController;
	}

	public static XboxController getDriverXbox() {
		return driverXboxController;
	}

	public static CommandXboxController getDriverCommandXbox() {
		return driverCommandXboxController;
	}

	/**
	 * Defines all the options for the autonomous delay
	 */
	static {
		for (double i = 0; i < 15; i += 0.25)
			autoDelayChooser.addOption(String.format("%.2f", i), i);

		autoDelayChooser.setDefaultOption("0.0", 0.0D);

		AUTO_TAB.add("Auto Start Delay", autoDelayChooser);
	}

	// The command configured to run during auto
	private Command autonomousCommand;

	@Override
	public void robotInit() {

		controllerChooser.addOption("Controller", "Controller");
		controllerChooser.addOption("JoySticks", "JoySticks");
		controllerChooser.setDefaultOption("Controller", "Controller");
		CONTROLLER_TAB.add("Controller", controllerChooser);

		Logger logger = Logger.getInstance();
		navX.reset();

		// Record metadata
		logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
		logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
		logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
		logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
		logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

		switch (BuildConstants.DIRTY) {
			case 0:
				logger.recordMetadata("GitDirty", "All changes committed");
				break;
			case 1:
				logger.recordMetadata("GitDirty", "Uncomitted changes");
				break;
			default:
				logger.recordMetadata("GitDirty", "Unknown");
				break;
		}

		// Set up data receivers & replay source
		if (Robot.isReal()) {
			// Running on a real robot, log to a USB stick
			logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
			logger.addDataReceiver(new NT4Publisher());
		} else {
			logger.addDataReceiver(new WPILOGWriter(""));
			logger.addDataReceiver(new NT4Publisher());
		}

		// Configure the joystick and controller bindings
		configureButtonBindings();

		Robot.swerve.setDefaultCommand(new DefaultDrive(true));
		Robot.claw.setDefaultCommand(new ClawOpwn());
		Robot.arm.setDefaultCommand(new DefaultIKControlCommand(true));
		// Robot.armtest.setDefaultCommand(new MoveArm());
		// Robot.arm.setDefaultCommand(new DefaultIKControlCommand(false));
		// Robot.photonvision.setDefaultCommand(new ReachCubeToNode());

		// add Autos to Shuffleboard
		Autonomous.init();
		AutonomousProgram.addAutosToShuffleboard();

		// Start Camera
		logger.start();
	}

	@Override
	public void autonomousInit() {
		// Chooses which auto we do from SmartDashboard
		autonomousCommand = AutonomousProgram.autoChooser.getSelected().construct();

		// Schedule the selected autonomous command group
		if (autonomousCommand != null) {
			CommandScheduler.getInstance().schedule(
					// To achieve the configured delay, use a sequential group that contains a wait
					// command
					new SequentialCommandGroup(
							new DriveWait(autoDelayChooser.getSelected()),
							new Homing(),
							autonomousCommand));
		}
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when teleop starts running.
		if (autonomousCommand != null)
			autonomousCommand.cancel();

		// CommandScheduler.getInstance().schedule(x);

	}

	@Override
	public void disabledInit() {
		swerve.resetToAbsoluteAngle();
	}

	@Override
	public void disabledPeriodic() {
		swerve.resetToAbsoluteAngle();
	}

	private void configureButtonBindings() {
		// operatorCommandXboxController.pov(0).onTrue(new
		// InstantCommand(operator::moveUp));
		// operatorCommandXboxController.pov(90).onTrue(new
		// InstantCommand(operator::moveRight));
		// operatorCommandXboxController.pov(180.33).onTrue(new
		// InstantCommand(operator::moveDown));
		// operatorCommandXboxController.pov(270).onTrue(new
		// InstantCommand(operator::moveLeft));
		// operatorCommandXboxController.y().onTrue(new
		// InstantCommand(operator::setPiece));
		// operatorCommandXboxController.b().onTrue(new
		// InstantCommand(operator::queuePlacement));
		// operatorCommandXboxController.x().onTrue(new
		// InstantCommand(operator::setNone));
		// operatorCommandXboxController.a().onTrue(new
		// InstantCommand(arm::resetEncodersNew));
		// operatorCommandXboxController.start().toggleOnFalse(new
		// ToggleOpen()).toggleOnTrue(new ToggleClose());

		// just testing manual angles
		// try reaching this configuration
		// operatorCommandXboxController.a().onTrue(new ReachAngles(20, 45, 0));
		operatorCommandXboxController.b().whileTrue(new ClawClose());
		// operatorCommandXboxController.a().toggleOnTrue(new InstantCommand(arm::toggleClaw));
		// operatorCommandXboxController.leftTrigger().whileTrue(new DefaultIKControlCommand(false))
		// 		.whileFalse(new DefaultMove());

		if (controllerChooser.getSelected().equals("Controller")) {
			// Reset Drive
			driverCommandXboxController.y()
					.onTrue(Commands.runOnce(navX::reset).andThen(Commands.runOnce(swerve::resetOdometry)));

			// Auto Level
			driverCommandXboxController.a().whileTrue(new AutolevelFeedforward())
					.whileFalse(new DefaultDrive(true));

			// Turn to angles
			driverCommandXboxController.pov(0).onTrue(Commands.runOnce(()->{arm.reach(Arrays.asList(0.0,0.0,0.0));}));
			driverCommandXboxController.pov(90).onTrue(Commands.runOnce(()->{arm.reach(Arrays.asList(0.0,0.0,0.0));}));
			driverCommandXboxController.pov(180).onTrue(Commands.runOnce(()->{arm.reach(Arrays.asList(-9.0,122.67,0.0));}));
			driverCommandXboxController.pov(270).onTrue(Commands.runOnce(()->{arm.reach(Arrays.asList(0.0,0.0,0.0));}));


			driverCommandXboxController.x().whileTrue(Commands.runOnce(swerve::lockWheels))
					.whileFalse(new DefaultDrive(true));

			driverCommandXboxController.x().onFalse(new DefaultDrive(true));
		} else {
			// new JoystickButton(leftJoystick, 3).onTrue(new InstantCommand(navX::reset));
			// new JoystickButton(leftJoystick, 3).onTrue(new
			// InstantCommand(swerve::resetOdometry));
			// new JoystickButton(leftJoystick, 4).onTrue(new AutolevelFeedforward());
			// new JoystickButton(leftJoystick, 4).onFalse(new DefaultDrive(true));
			// new JoystickButton(leftJoystick, 5).onTrue(new InstantCommand(swerve::stop));
			// new JoystickButton(leftJoystick, 5).onFalse(new DefaultDrive(true));
		}

	}

	/*
	 * This Robot is configured to run with the WPILib CommandScheduler.
	 * ⛔ Nothing should be handled in the below methods ⛔
	 */

	@Override
	public void robotPeriodic() {
		/*
		 * Runs the Scheduler. This is responsible for polling buttons, adding
		 * newly-scheduled
		 * commands, running already-scheduled commands, removing finished or
		 * interrupted commands,
		 * and running subsystem periodic() methods. This must be called from the
		 * robot's periodicd
		 * block in order for anything in the Command-based framework to work.
		 */

		// operatorCommandXboxController.a().toggleOnTrue(swerve.driveToNode());
		// NAVX_ANGLE.setDouble(Robot.getNavX().getAngle() % 360, 0);
		try {
			CommandScheduler.getInstance().run();
		} catch (Exception e) {
			System.out.println("It's shrey's fault");
			e.printStackTrace();
		}

	}
}
