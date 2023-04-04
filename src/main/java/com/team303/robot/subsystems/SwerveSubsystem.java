// Copyright (c) 2022 Team 303

package com.team303.robot.subsystems;

import static com.team303.robot.Robot.NAVX_ACCELERATION;
import static com.team303.robot.Robot.NAVX_Y_VELOCITY;
import static com.team303.robot.Robot.NAVX_ANGLE;
import static com.team303.robot.modules.OperatorGridModule.nodeSuperStateValues;

import java.awt.Point;
import java.awt.geom.Point2D;
import java.io.IOException;
import java.util.HashMap;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.team303.robot.Robot;
import com.team303.robot.RobotMap.Swerve;
import com.team303.robot.modules.OperatorGridModule.NodeSuperState;
import com.team303.robot.modules.PhotonvisionModule.CameraName;
// import com.team303.robot.commands.arm.AprilTagAlign;
import com.team303.swervelib.MkSwerveModuleBuilder;
import com.team303.swervelib.MotorType;
import com.team303.swervelib.SwerveModule;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import com.team303.robot.commands.arm.ReachPoint;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import static com.team303.robot.autonomous.AutonomousProgram.AUTO_TAB;

public class SwerveSubsystem extends SubsystemBase {

	/* odometry sim */
	private final double SECOND_TO_MS = 1000.0;
	private final Timer timer = new Timer();
	private double timeElapsed = 0;
	private double lastPeriodic = 0;
	public final AprilTagFieldLayout aprilTagField;
	private final Field2d field2d = new Field2d();
	private static final Vector<N3> swerveStandardDeviations = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));
	private static final Vector<N3> photonStandardDeviations = VecBuilder.fill(0.25, 0.25, 0);
	
	/* robot orientation chooser */
	public static final SendableChooser<String> orientationChooser = new SendableChooser<>();

	static {
		orientationChooser.setDefaultOption("Forward", "Forward");
		orientationChooser.addOption("Backward", "Forward");
		AUTO_TAB.add("Facing", orientationChooser).withPosition(2,0);
	}


	public PhotonPoseEstimator visionPoseEstimator;
	public SwerveDrivePoseEstimator poseEstimator;

	public static final ShuffleboardTab DRIVEBASE_TAB = Shuffleboard.getTab("Data");

	// private Rotation2d angle = new Rotation2d();
	public double angle = 0;
	private double positions[] = { 0, 0, 0, 0 };

	/* Field Sim */
	public static final Field2d field = new Field2d();

	/* Swerve Modules */
	private final SwerveModule leftFrontModule;
	private final SwerveModule rightFrontModule;
	private final SwerveModule leftBackModule;
	private final SwerveModule rightBackModule;

	/* Chasis Speeds */
	private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

	/* Kinematics */
	private final SwerveDriveKinematics kinematics;

	/* Odometry */
	SwerveDriveOdometry odometry;

	private Pose2d pose = new Pose2d(Swerve.STARTING_X, Swerve.STARTING_Y, new Rotation2d());

	public static final double MAX_VOLTAGE = 12.0;
	public static double MAX_DRIVE_SPEED = 0.75;

	/* Node Positions */
	public static double[] nodePositions = new double[9];
	public static final double START_NODE = 0.6540499999989841;
	public static final double DIFFERENCE = 0.53;
	public static final HashMap<String, Point2D.Double> nodePoints = new HashMap<String, Point2D.Double>();

	public SwerveSubsystem() {
		nodePoints.put("Top Cube", new Point2D.Double(Units.metersToInches(-1.01), Units.metersToInches(1.17)));
		nodePoints.put("Mid Cube", new Point2D.Double(Units.metersToInches(-0.58), Units.metersToInches(0.87)));
		nodePoints.put("Top Cone", new Point2D.Double(Units.metersToInches(-1.01), Units.metersToInches(0.90)));
		nodePoints.put("Mid Cone", new Point2D.Double(Units.metersToInches(-0.58), Units.metersToInches(0.60)));
		nodePoints.put("Bottom", new Point2D.Double(Units.metersToInches(-0.29), Units.metersToInches(0.05)));

		for (int i = 0; i < nodePositions.length; i++) {
			nodePositions[i] = START_NODE + i * DIFFERENCE;
		}

		timer.start();

		SmartDashboard.putData("FIELD SIM", field);

		kinematics = new SwerveDriveKinematics(
				new Translation2d(-Swerve.TRACKWIDTH / 2.0, -Swerve.WHEELBASE / 2.0),
				new Translation2d(-Swerve.TRACKWIDTH / 2.0, Swerve.WHEELBASE / 2.0),
				new Translation2d(Swerve.TRACKWIDTH / 2.0, -Swerve.WHEELBASE / 2.0),
				new Translation2d(Swerve.TRACKWIDTH / 2.0, Swerve.WHEELBASE / 2.0));

		leftFrontModule = new MkSwerveModuleBuilder()
				.withLayout(
						DRIVEBASE_TAB.getLayout("leftFrontModule", BuiltInLayouts.kGrid)
								.withSize(2, 4)
								.withPosition(4, 0))
				.withGearRatio(Swerve.MK4I_L1_LEFT_FRONT)
				.withDriveMotor(MotorType.NEO, Swerve.LEFT_FRONT_DRIVE_ID)
				.withSteerMotor(MotorType.NEO, Swerve.LEFT_FRONT_STEER_ID)
				.withSteerEncoderPort(Swerve.LEFT_FRONT_STEER_CANCODER_ID)
				.withSteerOffset(Swerve.LEFT_FRONT_STEER_OFFSET)
				.build();

		leftBackModule = new MkSwerveModuleBuilder()
				.withLayout(
						DRIVEBASE_TAB.getLayout("leftBackModule", BuiltInLayouts.kGrid)
								.withSize(2, 4)
								.withPosition(4, 0))
				.withGearRatio(Swerve.MK4I_L1_LEFT_BACK)
				.withDriveMotor(MotorType.NEO, Swerve.LEFT_BACK_DRIVE_ID)
				.withSteerMotor(MotorType.NEO, Swerve.LEFT_BACK_STEER_ID)
				.withSteerEncoderPort(Swerve.LEFT_BACK_STEER_CANCODER_ID)
				.withSteerOffset(Swerve.LEFT_BACK_STEER_OFFSET)
				.build();

		rightFrontModule = new MkSwerveModuleBuilder()
				.withLayout(
						DRIVEBASE_TAB.getLayout("rightFrontModule", BuiltInLayouts.kGrid)
								.withSize(2, 4)
								.withPosition(4, 0))
				.withGearRatio(Swerve.MK4I_L1_RIGHT_FRONT)
				.withDriveMotor(MotorType.NEO, Swerve.RIGHT_FRONT_DRIVE_ID)
				.withSteerMotor(MotorType.NEO, Swerve.RIGHT_FRONT_STEER_ID)
				.withSteerEncoderPort(Swerve.RIGHT_FRONT_STEER_CANCODER_ID)
				.withSteerOffset(Swerve.RIGHT_FRONT_STEER_OFFSET)
				.build();

		rightBackModule = new MkSwerveModuleBuilder()
				.withLayout(
						DRIVEBASE_TAB.getLayout("rightBackModule", BuiltInLayouts.kGrid)
								.withSize(2, 4)
								.withPosition(4, 0))
				.withGearRatio(Swerve.MK4I_L1_RIGHT_BACK)
				.withDriveMotor(MotorType.NEO, Swerve.RIGHT_BACK_DRIVE_ID)
				.withSteerMotor(MotorType.NEO, Swerve.RIGHT_BACK_STEER_ID)
				.withSteerEncoderPort(Swerve.RIGHT_BACK_STEER_CANCODER_ID)
				.withSteerOffset(Swerve.RIGHT_BACK_STEER_OFFSET)
				.build();

		resetToAbsoluteAngle();

		if (Robot.isReal()) {
			odometry = new SwerveDriveOdometry(
				// consider making angle value negative
					kinematics, Rotation2d.fromDegrees(Robot.navX.getAngle()),
					new SwerveModulePosition[] {
							leftFrontModule.getPosition(),
							rightFrontModule.getPosition(),
							leftBackModule.getPosition(),
							rightBackModule.getPosition(),

					}, pose);
		} else {
			odometry = new SwerveDriveOdometry(
					kinematics, Rotation2d.fromDegrees(0),
					new SwerveModulePosition[] {
							new SwerveModulePosition(0, new Rotation2d()),
							new SwerveModulePosition(0, new Rotation2d()),
							new SwerveModulePosition(0, new Rotation2d()),
							new SwerveModulePosition(0, new Rotation2d())
					}, pose);
		}

		AprilTagFieldLayout initialLayout;

		try {
			initialLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
			var alliance = DriverStation.getAlliance();
			System.out.println(alliance.name());
			initialLayout.setOrigin(alliance == Alliance.Blue ? OriginPosition.kBlueAllianceWallRightSide
					: OriginPosition.kRedAllianceWallRightSide);
		} catch (IOException e) {
			DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
			initialLayout = null;
		}

		aprilTagField = initialLayout;
		// photonvision pose estimator

		// if (Robot.isReal()) {
		// 	visionPoseEstimator = new PhotonPoseEstimator(aprilTagField, PoseStrategy.MULTI_TAG_PNP,
		// 			Robot.photonvision.getCamera(CameraName.CAM1), new Transform3d(new Translation3d(), new Rotation3d()));
		// 	visionPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
		// }
		poseEstimator = new SwerveDrivePoseEstimator(
				kinematics,
				new Rotation2d(),
				new SwerveModulePosition[] {
						new SwerveModulePosition(0.0, new Rotation2d()),
						new SwerveModulePosition(0.0, new Rotation2d()),
						new SwerveModulePosition(0.0, new Rotation2d()),
						new SwerveModulePosition(0.0, new Rotation2d()),
				},
				pose,
				swerveStandardDeviations,
				photonStandardDeviations);
		DRIVEBASE_TAB.add("Pose", toString()).withPosition(0, 0).withSize(2, 0);
		DRIVEBASE_TAB.add("Field", field2d).withPosition(2, 0).withSize(6, 4);

		// var thread = 
		// 		new Thread(
		// 			() -> {
		// 				if (aprilTagField == null) {
		// 					return;
		// 				}
		// 				while (!Thread.currentThread().isInterrupted()) {
		// 					poseEstimator.update(
		// 						Rotation2d.fromDegrees(-Robot.navX.getAngle()),
		// 						Robot.swerve.getModulePositions());
		// 					Optional<EstimatedRobotPose> result = getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
		// 					if (result.isPresent()) {
		// 						EstimatedRobotPose visionPoseEstimate = result.get();
		// 						poseEstimator.addVisionMeasurement(visionPoseEstimate.estimatedPose.toPose2d(),
		// 								visionPoseEstimate.timestampSeconds);
		// 					}
		// 					try {
		// 					  Thread.sleep(Swerve.THREAD_SLEEP_DURATION_MS);
		// 					} catch (InterruptedException e) {
		// 					  Thread.currentThread().interrupt();
		// 					}
		// 				  }
		// 			}
		// 		);
		// thread.setDaemon(true);
		// thread.start();
	}

	public void resetToAbsoluteAngle() {
		leftBackModule.resetToAbsolute();
		leftFrontModule.resetToAbsolute();
		rightBackModule.resetToAbsolute();
		rightFrontModule.resetToAbsolute();
	}

	public void setMaxSpeed(double maxSpeed) {
		MAX_DRIVE_SPEED = maxSpeed;
	}

	// return kinematics instance
	public SwerveDriveKinematics getKinematics() {
		return kinematics;
	}

	// return current positition and angle
	public Pose2d getPose() {
		return pose;
	}

	public SwerveDriveOdometry getOdometry() {
		return odometry;
	}

	public void resetOdometry() {
		SwerveModuleState[] state = kinematics.toSwerveModuleStates(chassisSpeeds);
		SwerveModulePosition[] newSwervePositions;

		if (Robot.isReal()) {
			newSwervePositions = new SwerveModulePosition[] {
					leftFrontModule.getPosition(),
					rightFrontModule.getPosition(),
					leftBackModule.getPosition(),
					rightBackModule.getPosition()
			};
		} else {
			newSwervePositions = new SwerveModulePosition[] {
					new SwerveModulePosition(positions[0], state[0].angle),
					new SwerveModulePosition(positions[1], state[1].angle),
					new SwerveModulePosition(positions[2], state[2].angle),
					new SwerveModulePosition(positions[3], state[3].angle)
			};
		}
		System.out.println("Resetting");

		if (orientationChooser.getSelected().equals("Forward")) {			
			odometry.resetPosition(Robot.navX.getRotation2d(), newSwervePositions,
					new Pose2d(0, 0, new Rotation2d()));
		} else {
			odometry.resetPosition(Robot.navX.getRotation2d(), newSwervePositions,
					new Pose2d(0, 0, new Rotation2d(Math.PI)));			
		}
		this.angle = 0;

	}

	public void resetOdometry(Pose2d pose) {
		if (Robot.isReal()) {
			odometry.resetPosition(Robot.navX.getRotation2d(), getModulePositions(), pose);
		} else {
			odometry.resetPosition(pose.getRotation(), getModulePositions(), pose);
		}
	}

	public SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[] {
				leftFrontModule.getPosition(),
				rightFrontModule.getPosition(),
				leftBackModule.getPosition(),
				rightBackModule.getPosition(),
		};
	}

	public void drive(Translation2d translation, double rotation, boolean fieldOriented) {
		rotation *= Swerve.ROTATION_CONSTANT;

		if (fieldOriented && Robot.isReal()) {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), -rotation,
					Rotation2d.fromDegrees(-Robot.navX.getAngle()));
		} else if (fieldOriented && !Robot.isReal()) {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), -rotation,
					Rotation2d.fromRadians(angle));
		} else {
			chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
		}

		drive(kinematics.toSwerveModuleStates(chassisSpeeds));
	}

	public void offCenterDrive(Translation2d translation, double rotation, Translation2d centerOfRotation,
			boolean fieldOriented) {
		rotation *= Swerve.ROTATION_CONSTANT;

		if (Math.abs(rotation) <= 1) {
			rotation = 0.0;
		}

		if (fieldOriented) {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
					Rotation2d.fromDegrees(Robot.navX.getAngle()));
		} else {
			chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
		}

		drive(kinematics.toSwerveModuleStates(chassisSpeeds, centerOfRotation));
	}

	// generic drive method

	public void lockWheels() {
		// lockWheels
		leftFrontModule.set(0, Math.PI / 2);
		rightFrontModule.set(0, Math.PI / 2);
		leftBackModule.set(0, Math.PI / 2);
		rightBackModule.set(0, Math.PI / 2);
	}

	public void drive(SwerveModuleState[] state) {
		chassisSpeeds = kinematics.toChassisSpeeds(state);
		// map speed of swerve modules to voltage
		leftFrontModule.set(state[0].speedMetersPerSecond / Swerve.MAX_VELOCITY * MAX_VOLTAGE,
				state[0].angle.getRadians());
		rightFrontModule.set(state[1].speedMetersPerSecond / Swerve.MAX_VELOCITY * MAX_VOLTAGE,
				state[1].angle.getRadians());
		leftBackModule.set(state[2].speedMetersPerSecond / Swerve.MAX_VELOCITY * MAX_VOLTAGE,
				state[2].angle.getRadians());
		rightBackModule.set(state[3].speedMetersPerSecond / Swerve.MAX_VELOCITY * MAX_VOLTAGE,
				state[3].angle.getRadians());
	}

	public Pose2d getRobotPose() {
		return poseEstimator.getEstimatedPosition();
	}

	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
		visionPoseEstimator.setReferencePose(prevEstimatedRobotPose);
		return visionPoseEstimator.update();
	}

	// Sets the pose estimation to a new pose
	public void setRobotPose(Pose2d newPose) {
		poseEstimator.resetPosition(
				Rotation2d.fromDegrees(Robot.navX.getAngle()),
				Robot.swerve.getModulePositions(),
				newPose);
	}

	public void stop() {
		leftFrontModule.set(0, 0);
		rightFrontModule.set(0, 0);
		leftBackModule.set(0, 0);
		rightBackModule.set(0, 0);
	}
	// Assuming this method is part of a drivetrain subsystem that provides the
	// necessary methods

	@Override
	public void periodic() {
		double triggerPressure = Robot.driverController.getLeftTriggerAxis();
		if (Robot.driverController.b().getAsBoolean()) {
			MAX_DRIVE_SPEED = 1.0;
		} else if (triggerPressure > 0.01) {
			// Map max speed from 0-100 to 50-20
			MAX_DRIVE_SPEED = triggerPressure * -3 / 10 + 0.5;
		} else if (!Robot.driverController.b().getAsBoolean()) {
			MAX_DRIVE_SPEED = 0.75;
		}

		if (Robot.isReal()) {
			// Update Poses
			pose = odometry.update(
					Rotation2d.fromDegrees(-Robot.navX.getAngle()),
					new SwerveModulePosition[] {
							leftFrontModule.getPosition(),
							rightFrontModule.getPosition(),
							leftBackModule.getPosition(),
							rightBackModule.getPosition(),
					});
		} else {
			SwerveModuleState[] state = kinematics.toSwerveModuleStates(chassisSpeeds);
			timeElapsed = (timer.get() - lastPeriodic) * SECOND_TO_MS;
			angle += chassisSpeeds.omegaRadiansPerSecond / SECOND_TO_MS * timeElapsed;
			positions[0] += state[0].speedMetersPerSecond / SECOND_TO_MS * timeElapsed;
			positions[1] += state[1].speedMetersPerSecond / SECOND_TO_MS * timeElapsed;
			positions[2] += state[2].speedMetersPerSecond / SECOND_TO_MS * timeElapsed;
			positions[3] += state[3].speedMetersPerSecond / SECOND_TO_MS * timeElapsed;

			pose = odometry.update(Rotation2d.fromRadians(angle),
					new SwerveModulePosition[] {
							new SwerveModulePosition(positions[0], state[0].angle),
							new SwerveModulePosition(positions[1], state[1].angle),
							new SwerveModulePosition(positions[2], state[2].angle),
							new SwerveModulePosition(positions[3], state[3].angle),
					});
		}
		// poseEstimator.update(
		// Rotation2d.fromDegrees(-Robot.navX.getAngle()),
		// Robot.swerve.getModulePositions());
		field2d.setRobotPose(getRobotPose());

		lastPeriodic = timer.get();

		NAVX_Y_VELOCITY.setDouble(Robot.navX.getRawGyroZ(), 0);
		NAVX_ACCELERATION.setDouble(Robot.navX.getRawAccelX());
		NAVX_ANGLE.setDouble(Robot.navX.getAngle());
		SmartDashboard.putNumber("Pitch acceleration", -Robot.navX.getRawAccelY());
		SmartDashboard.putNumber("Pitch ", -Robot.navX.getPitch());
		Logger.getInstance().recordOutput("Swerve Module States", kinematics.toSwerveModuleStates(chassisSpeeds));
		Logger.getInstance().recordOutput("Acceleration X", Robot.navX.getWorldLinearAccelX());
		Logger.getInstance().recordOutput("Acceleration Y", Robot.navX.getWorldLinearAccelY());
		Logger.getInstance().recordOutput("Acceleration Z", Robot.navX.getWorldLinearAccelZ());
		Logger.getInstance().recordOutput("Velocity X", Robot.navX.getVelocityX());
		Logger.getInstance().recordOutput("Raw Acceleration", Robot.navX.getRawAccelX());
		Logger.getInstance().recordOutput("Odometry", pose);
		SmartDashboard.putNumber("Pose X", pose.getX());
		SmartDashboard.putNumber("Pose Y", pose.getY());
		// Logger.getInstance().recordOutput("Odometry 2", getRobotPose());

	}

	public CommandBase followTrajectoryCommand(PathPlannerTrajectory traj) {
		// Reset odometry for the first path you run during auto
		return new PPSwerveControllerCommand(
				traj,
				Robot.swerve::getPose, // Pose supplier
				Robot.swerve.getKinematics(), // SwerveDriveKinematics
				new PIDController(0, 0.2, 0), // X controller. Tune these values for your robot. Leaving them 0
												// will only use feedforwards.
				new PIDController(0, 0.2, 0), // Y controller (usually the same values as X controller)
				new PIDController(1, 0.13, 0), // Rotation controller. Tune these values for your robot. Leaving
												// them 0 will only use feedforwards.
				Robot.swerve::drive, // Module states consumer
				true, // Should the path be automatically mirrored depending on alliance color.
				Robot.swerve);
	}

	public CommandBase driveToPose(Pose2d point1, Pose2d point2, Pose2d... poses) {

		PathPoint[] points = new PathPoint[poses.length];

		for (int i = 0; i < poses.length; i++) {
			points[i] = new PathPoint(poses[i].getTranslation(), poses[i].getRotation());
		}

		PathPlannerTrajectory trajectory = PathPlanner.generatePath(
				new PathConstraints(4, 3),
				new PathPoint(point1.getTranslation(), point1.getRotation()),
				new PathPoint(point2.getTranslation(), point2.getRotation()),
				points);

		return followTrajectoryCommand(trajectory);
	}

	public CommandBase driveToPose(Pose2d point1, Pose2d point2) {

		PathPlannerTrajectory trajectory = PathPlanner.generatePath(
				new PathConstraints(4, 3),
				new PathPoint(point1.getTranslation(), point1.getRotation()),
				new PathPoint(
						new Translation2d(
								point1.getTranslation()
										.plus(point2.getTranslation().div(2).minus(new Translation2d(0, 0.5))).getX(),
								point2.getTranslation().getY()),
						point2.getRotation()),
				new PathPoint(point2.getTranslation(), point2.getRotation()));

		return followTrajectoryCommand(trajectory);
	}

	public CommandBase driveToNode() {

		Point posePoint = new Point(0, 0);

		for (int i = 0; i < nodeSuperStateValues.length; i++) {
			for (int j = 0; j < nodeSuperStateValues[i].length; j++) {
				if (nodeSuperStateValues[i][j] == NodeSuperState.QUEUED.value) {
					posePoint = new Point(i, j);
				}
			}
		}

		String reachNode = "Bottom";

		if (posePoint.x == 0 && posePoint.y % 3 == 2) {
			reachNode = "Top Cube";
		} else if (posePoint.x == 1 && posePoint.y % 3 == 2) {
			reachNode = "Mid Cube";
		} else if (posePoint.x == 0) {
			reachNode = "Top Cone";
		} else if (posePoint.x == 1) {
			reachNode = "Mid Cone";
		}

		return new ParallelCommandGroup(
				driveToPose(Robot.swerve.getPose(), new Pose2d(1.75, nodePositions[posePoint.y], new Rotation2d()))
		// new ReachPoint(nodePoints.get(reachNode).x, nodePoints.get(reachNode).y)
		);
	}

	// @Override
	// public String toString() {
	// var pose = getRobotPose();
	// return String.format("(%.2f, %.2f) %.2f degrees",
	// pose.getX(),
	// pose.getY(),
	// pose.getRotation().getDegrees());
	// }
}