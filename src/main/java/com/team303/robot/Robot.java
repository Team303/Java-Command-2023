package com.team303.robot;

import java.util.Arrays;
import java.util.HashMap;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.kauailabs.navx.frc.AHRS;
import com.team303.robot.RobotMap.IOConstants;
import com.team303.robot.RobotMap.LED;
import com.team303.robot.autonomous.Autonomous;
import com.team303.robot.autonomous.AutonomousProgram;
import com.team303.robot.commands.arm.DefaultArm;
import com.team303.robot.commands.arm.HomeArm;
import com.team303.robot.commands.arm.HomeArmContinuous;
import com.team303.robot.commands.claw.DefaultClaw;
import com.team303.robot.commands.drive.AutoLevelBasic;
import com.team303.robot.commands.drive.AutolevelFeedforward;
import com.team303.robot.commands.drive.DefaultDrive;
import com.team303.robot.commands.drive.DriveWait;
import com.team303.robot.modules.OperatorGridModule;
import com.team303.robot.modules.PhotonvisionModule;
import com.team303.robot.modules.UltrasonicModule;
import com.team303.robot.subsystems.ArmSubsystem;
import com.team303.robot.subsystems.ArmTestSubsystem;
import com.team303.robot.subsystems.ClawSubsystem;
import com.team303.robot.subsystems.IntakeSubsystem;
import com.team303.robot.subsystems.SwerveSubsystem;
import com.team303.robot.subsystems.LEDSubsystem;
import com.team303.robot.subsystems.ManipulatorSubsystem;
import com.team303.robot.commands.arm.DefaultIKControlCommand;
import com.team303.robot.commands.arm.ReachPoint;
import com.team303.robot.commands.arm.ReachPointContinuous;
import com.team303.robot.commands.intake.DefaultIntake;
import com.team303.robot.subsystems.ClawSubsystem.ClawState;
import com.team303.robot.subsystems.IntakeSubsystem.IntakeState;
import com.team303.robot.subsystems.ManipulatorSubsystem.GamePieceType;
import frc.robot.BuildConstants;
import com.team303.robot.commands.arm.ElbowUp;
import com.team303.robot.commands.arm.ReachAngles;
import static com.team303.robot.subsystems.ClawSubsystem.clawStateChooser;
import static com.team303.robot.subsystems.ClawSubsystem.clawModeChooser;
import static com.team303.robot.subsystems.IntakeSubsystem.intakeStateChooser;
import static com.team303.robot.subsystems.IntakeSubsystem.intakeModeChooser;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.cameraserver.CameraServer;

public class Robot extends LoggedRobot {

	/* roboRIO Sensors */
	public static final AHRS navX = new AHRS();

	/* Robot Modules */
	public static final PhotonvisionModule photonvision = null; // new Photonvision();
	public static final UltrasonicModule ultrasonic = null; // new Ultrasonic(0, 4);
	public static final OperatorGridModule operatorGrid = new OperatorGridModule();

	/* Robot Subsystems */
	public static final SwerveSubsystem swerve = new SwerveSubsystem();
	public static final ManipulatorSubsystem manipulator = new ClawSubsystem();
	public static final ArmSubsystem arm = new ArmSubsystem();
	// public static final ClawSubsystem claw = null; //new ClawSubsystem();
	// public static final IntakeSubsystem intake = new IntakeSubsystem();
	public static final ArmTestSubsystem armTest = null; // new ArmTest();
	public static final LEDSubsystem ledStrip = null; // new LEDSubsystem();

	/* Robot IO Controls */
	public static final CommandXboxController operatorController = new CommandXboxController(
			IOConstants.OPERATOR_CONTROLLER);
	public static final CommandXboxController driverController = new CommandXboxController(
			IOConstants.DRIVER_CONTROLLER);

	/* Shufflebaord Tabs */
	public static final ShuffleboardTab CONTROLLER_TAB = Shuffleboard.getTab("Controller");
	public static final ShuffleboardTab NAVX_TAB = Shuffleboard.getTab("NavX");

	public static final GenericEntry NAVX_Y_VELOCITY = NAVX_TAB.add("Y Velocity", 0).getEntry();
	public static final GenericEntry NAVX_ACCELERATION = NAVX_TAB.add("Acceleration", 0).getEntry();
	public static final GenericEntry NAVX_ANGLE = NAVX_TAB.add("NavX Angle", 0).getEntry();

	/* Robot State */
	public static final Color ALLIANCE_COLOR = DriverStation.getAlliance() == Alliance.Blue ? LED.RED : LED.BLUE;

	public static enum HeldObject {
		NONE,
		CUBE,
		CONE
	}

	public static HeldObject heldObject = HeldObject.NONE;

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

	/* Currently running auto routine */

	private Command autonomousCommand;

	@Override
	public void robotInit() {
		Logger logger = Logger.getInstance();
		navX.reset();
		CameraServer.startAutomaticCapture();

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
		Robot.manipulator.setDefaultCommand(new DefaultClaw());
		// Robot.intake.setDefaultCommand(new DefaultIntake());

		// add Autos to Shuffleboard
		Autonomous.init();
		AutonomousProgram.addAutosToShuffleboard();

		// Start Camera
		logger.start();
	}

	@Override
	public void autonomousInit() {
		navX.reset();
		swerve.resetOdometry();

		// Dont do IK during auto
		Robot.arm.removeDefaultCommand();
		// Command startCommand = new SequentialCommandGroup(new ElbowUp(30), new
		// HomeArm());
		// if (manipulator instanceof ClawSubsystem) {
		ClawSubsystem stateClaw = (ClawSubsystem) manipulator;
		stateClaw.state = clawStateChooser.getSelected();
		stateClaw.mode = clawModeChooser.getSelected();
		// startCommand = stateClaw.state==ClawState.OPEN ? startCommand = new HomeArm()
		// :
		// new SequentialCommandGroup(new ElbowUp(45), new HomeArm());
		// } else {
		// IntakeSubsystem stateIntake = (IntakeSubsystem) manipulator;
		// stateIntake.state = intakeStateChooser.getSelected();
		// stateIntake.mode = intakeModeChooser.getSelected();
		// // startCommand = Commands.none();
		// }

		// Chooses which auto we do from Shuffleboard
		Command autonomousRoutine = AutonomousProgram.constructSelectedRoutine();

		// Home the arm while waiting for the drivebase delay
		Command delay = new ParallelCommandGroup(
				new HomeArm(),
				new DriveWait(AutonomousProgram.getAutonomousDelay()));

		// Schedule the selected autonomous command group
		if (autonomousRoutine != null) {
			// Run the delay/home and the selected routine sequentially
			this.autonomousCommand = new SequentialCommandGroup(
					delay,
					autonomousRoutine);
		} else {
			this.autonomousCommand = delay;
		}

		// Schedule the combined command group
		CommandScheduler.getInstance().schedule(this.autonomousCommand);
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when teleop starts running.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}

		// Robot.arm.setDefaultCommand(new DefaultIKControlCommand(false));
		// if (operatorController.getLeftTriggerAxis() < 0.9) {
		Robot.arm.setDefaultCommand(new DefaultIKControlCommand(false));
		// } else {
		// Robot.arm.setDefaultCommand(new DefaultArm());
		// }
	}

	@Override
	public void disabledInit() {
		swerve.resetToAbsoluteAngle();
	}

	@Override
	public void disabledPeriodic() {
		// While disabled, continuously reset the angle of the swerve modules
		swerve.resetToAbsoluteAngle();
	}

	private void configureButtonBindings() {
		/* Operator Controls */

		// Custo m grid widget button bindings
		// operat\9\n9n9nnnnnnnnnnnnnnn\nnnnnnnnnnnnnnnnnnnnnnnnnn9

		// operatorController.pov(90).onTrue(new
		// InstantCommand(operatorGrid::moveRight));
		// operatorController.pov(180).onTrue(new
		// InstantCommand(operatorGrid::moveDown));
		// operatorController.pov(270).onTrue(new
		// InstantCommand(operatorGrid::moveLeft));

		// operatorController.y().onTrue(new InstantCommand(operatorGrid::setPiece));
		// operatorController.x().onTrue(new
		// InstantCommand(operatorGrid::queuePlacement));

		// operatorController.y().onTrue(new InstantCommand(operatorGrid::setPiece));
		// operatorController.x().onTrue(new
		// InstantCommand(operatorGrid::queuePlacement));

		// Claw Control
		// operatorController.y().onTrue(new InstantCommand(operatorGrid::setPiece));
		// operatorController.x().onTrue(new
		// InstantCommand(operatorGrid::queuePlacement));

		// Claw Control
		operatorController.b().onTrue(new InstantCommand(manipulator::nextState));
		operatorController.a().onTrue(new InstantCommand(manipulator::toggleMode));
		// operatorController.rightBumper().onTrue(Commands.runOnce(() ->
		// arm.setClawAngleConstraint((float)Math.toRadians(0)))).onFalse(Commands.runOnce(()
		// -> arm.setClawAngleConstraint((float)Math.toRadians(-90))));

		// Top Cone
		operatorController.pov(0).whileTrue(new ReachPoint(78.974, 92.246).repeatedly());
		// Substation
		operatorController.pov(90).whileTrue(new ReachPoint(86.356, 105.204).repeatedly());
		// Mid Cone
		operatorController.pov(180).whileTrue(new ReachPoint(102.233, 86.082).repeatedly());
		// Bottom
		operatorController.pov(270)
				.onTrue(new SequentialCommandGroup(new HomeArm(), new ReachPoint(29.178, 82.041)));

		// operatorController.x().whileTrue(new HomeArmContinuous());
		operatorController.x().toggleOnTrue(new HomeArmContinuous(0.2, 0.2, 0.3));
		// operatorController.y().toggleOnTrue(new ReachAngles(-19.5, 160.0, -86.0));

		// driverController.pov(0).onTrue(new ReachPoint(36, 0));

		// Lock swerve wheels while x is held
		/* Driver Controls */

		driverController.x().whileTrue(Commands.runOnce(swerve::lockWheels));

		// Reset field oriented drive when y is pressed
		driverController.y()
				.onTrue(Commands.runOnce(navX::reset).andThen(Commands.runOnce(swerve::resetOdometry)));

		// Auto level while a is held
		driverController.x().whileTrue(new AutoLevelBasic());

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

		try {
			CommandScheduler.getInstance().run();
		} catch (Exception e) {
			e.printStackTrace();
		}

	}
}
