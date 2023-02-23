package com.team303.robot;

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
import com.team303.robot.commands.drive.DefaultDrive;
import com.team303.robot.commands.drive.DriveWait;
import com.team303.robot.commands.drive.FollowTrajectory;
import com.team303.robot.commands.drive.TurnToAngle;
import com.team303.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.team303.robot.subsystems.ArmTest;
import com.team303.robot.commands.MoveArm;
import com.team303.robot.modules.Photonvision;
import com.team303.robot.commands.drive.AutolevelFeedforward;
import com.team303.robot.commands.arm.ReachCubeToNode;
//import com.team303.robot.modules.Photonvision;

import com.team303.robot.commands.drive.AutolevelFeedforward;
import com.team303.robot.commands.drive.AutolevelPID;

public class Robot extends LoggedRobot {

	/* RoboRio Sensors */
	private static final AHRS navX = new AHRS();
	/* Robot Subsystems */
	public static final SwerveSubsystem swerve = null; // new SwerveSubsystem();
	public static final ArmTest arm = null; //new ArmTest();
	public static final Photonvision photonvision = new Photonvision();

	/* Robot IO Controls */
	private static final Joystick leftJoystick = new Joystick(IOConstants.LEFT_JOYSTICK_ID);
	private static final Joystick rightJoystick = new Joystick(IOConstants.RIGHT_JOYSTICK_ID);
	private static final CommandXboxController xboxController = new CommandXboxController(IOConstants.OPERATOR_CONTROLLER);

	/* Shufflebaord Tabs */
	public static final ShuffleboardTab AUTO_TAB = Shuffleboard.getTab("Autonomous");

	/* Shuffleboard Choosers */
	public static SendableChooser<Double> autoDelayChooser = new SendableChooser<>();
	public static HashMap<String, Command> eventMap = new HashMap<>();

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
	public static final int ALLIANCE_SUBSTATION_ID = DriverStation.getAlliance() == Alliance.Blue ? SubstationFiducialID.RED.fiducialId : SubstationFiducialID.BLUE.fiducialId;
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

	public static CommandXboxController getXbox() {
		return xboxController;
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
	private Command autonomousCommand;;

	@Override
	public void robotInit() {

		Logger logger = Logger.getInstance();

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

		// Robot.swerve.setDefaultCommand(new DefaultDrive(true));
		// Robot.arm.setDefaultCommand(new MoveArm());
		Robot.photonvision.setDefaultCommand(new ReachCubeToNode());

		// Place event markers here
		// eventMap.put("marker1", new PrintCommand("Passed marker 1"));
		// add Autos to Shuffleboard
		// Autonomous.init();
		// //System.out.println("Auton Init");
		// AutonomousProgram.addAutosToShuffleboard();

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
							autonomousCommand));
		}

		// Match LEDs color to team
		//CommandScheduler.getInstance().schedule(new LEDSolidColor(allianceColor));
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when teleop starts running.
		if (autonomousCommand != null)
			autonomousCommand.cancel();

		// Match LEDs color to team
		//CommandScheduler.getInstance().schedule(new LEDSolidColor(allianceColor));

	}

	@Override
	public void disabledInit() {
		// Change LED color to signify disabled state
		//CommandScheduler.getInstance().schedule(new LEDSolidColor(LED.DISABLED_COLOR));
	}

	@Override
	public void disabledPeriodic() {
	}

	private void configureButtonBindings() {
		// xboxController.y().onTrue(new InstantCommand(navX::reset));
		// xboxController.x().onTrue(new InstantCommand(swerve::resetOdometry));
		// //xboxController.a().onTrue(new AutolevelFeedforward());
		// //xboxController.a().onFalse(new DefaultDrive(true));
		// xboxController.b().onTrue(new InstantCommand(swerve::stop));
		// xboxController.b().onFalse(new DefaultDrive(true));


		// new JoystickButton(leftJoystick, 3).onTrue(new InstantCommand(navX::reset));
		// new JoystickButton(leftJoystick, 3).onTrue(new InstantCommand(swerve::resetOdometry));
		// new JoystickButton(leftJoystick, 4).onTrue(new AutolevelFeedforward());
		// new JoystickButton(leftJoystick, 4).onFalse(new DefaultDrive(true));
		// new JoystickButton(leftJoystick, 5).onTrue(new InstantCommand(swerve::stop));
		// new JoystickButton(leftJoystick, 5).onFalse(new DefaultDrive(true));
	}

	@Override
	public void simulationInit() {
		System.out.println(WPILibVersion.Version);
		// set default commands


		// Path Weaver Trajectory
		try {
			Trajectory trajectory = FollowTrajectory.convert("output/Test.wpilib.json");

			// Push the trajectory to Field2d.
			//SwerveSubsystem.field.getObject("traj").setTrajectory(trajectory);
		} catch (Exception e) {
			e.printStackTrace();
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
		 * robot's periodic
		 * block in order for anything in the Command-based framework to work.
		 */

		/* 
		SmartDashboard.putNumber("X crosshair", Limelight.getLimelight().getEntry("tx").getDouble(0.0));
		SmartDashboard.putNumber("Y crosshair", Limelight.getLimelight().getEntry("ty").getDouble(0.0));
		SmartDashboard.putNumber("Num Targets", Limelight.getLimelight().getEntry("tv").getDouble(0.0));
		SmartDashboard.putNumber("Target Area", Limelight.getLimelight().getEntry("ta").getDouble(0.0));*/

		CommandScheduler.getInstance().run();
	}
}
