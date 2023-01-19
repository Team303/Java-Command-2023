// Copyright (c) 2022 Team 303

package com.team303.robot.subsystems;

import java.lang.reflect.GenericArrayType;

import com.ctre.phoenix.sensors.CANCoder;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.Timer;
import com.team303.robot.Robot;
import com.team303.robot.RobotMap.Swerve;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {

	/* odometry sim */
	private final double DELAY_MS = 100.0;
    private final double SECOND_TO_MS = 1000.0;
    private final Timer timer = new Timer();
	private double last = 0;

	//private Rotation2d angle = new Rotation2d();
	private double angle = 0;
	private double positions[] = {0, 0, 0, 0};

	/* Field Sim */
	public static final Field2d field = new Field2d();

	/* ShuffleBoard */
	public static final ShuffleboardTab DRIVEBASE_TAB = Shuffleboard.getTab("Drivebase");
	//public static final ShuffleboardTab FIELD_TAB = Shuffleboard.getTab("Drivebase");

	public static final GenericEntry NAVX_ANGLE_ENTRY = DRIVEBASE_TAB.add("NavX Angle", 0).getEntry();
	public static final GenericEntry NAVX_RATE_ENTRY = DRIVEBASE_TAB.add("NavX Rate", 0).getEntry();
	public static final GenericEntry POSITION_X_ENTRY = DRIVEBASE_TAB.add("Position X", 0).getEntry();
	public static final GenericEntry POSITION_Y_ENTRY = DRIVEBASE_TAB.add("Position Y", 0).getEntry();
	public static final GenericEntry LEFT_FRONT_STEER_ANGLE_ENTRY = DRIVEBASE_TAB.add("Front Left Steer Angle", 0).getEntry();
	public static final GenericEntry RIGHT_FRONT_STEER_ANGLE_ENTRY = DRIVEBASE_TAB.add("Front Right Steer Angle", 0).getEntry();
	public static final GenericEntry LEFT_BACK_STEER_ANGLE_ENTRY = DRIVEBASE_TAB.add("Back Left Steer Angle", 0).getEntry();
	public static final GenericEntry RIGHT_BACK_STEER_ANGLE_ENTRY = DRIVEBASE_TAB.add("Back Right Steer Angle", 0).getEntry();
	public static final GenericEntry DRIVE_ENCODER_ENTRY = DRIVEBASE_TAB.add("Average Encoders", 0).getEntry();
	//public static final ComplexWidget FIELD_SIM_ENTRY = DRIVEBASE_TAB.add("FIELD SIM", field);

	/*Swerve Modules*/

	private final SwerveModule leftFrontModule;
	private final SwerveModule rightFrontModule;
	private final SwerveModule leftBackModule;
	private final SwerveModule rightBackModule;

	/*Drive Encoders*/

	private final CANCoder leftFrontEncoder;
	private final CANCoder leftBackEncoder;
	private final CANCoder rightFrontEncoder;
	private final CANCoder rightBackEncoder;

	/*Chasis Speeds*/
	private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

	/* Kinematics */
	private final SwerveDriveKinematics kinematics;

	/* Odometry */
	SwerveDriveOdometry odometry;

	private Pose2d pose = new Pose2d();
	private SwerveDrivePoseEstimator poseEstimator;

	public static final double MAX_VOLTAGE = 12.0;

	public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
		SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
		SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

	private static SwerveSubsystem instance = new SwerveSubsystem();

	private SwerveSubsystem() {
		
		timer.start();

		SmartDashboard.putData("FIELD SIM", field);

		leftFrontEncoder = new CANCoder(Swerve.LEFT_FRONT_DRIVE_CANCODER_ID);
		leftBackEncoder = new CANCoder(Swerve.LEFT_BACK_DRIVE_CANCODER_ID);
		rightFrontEncoder = new CANCoder(Swerve.RIGHT_FRONT_DRIVE_CANCODER_ID);
		rightBackEncoder = new CANCoder(Swerve.RIGHT_BACK_DRIVE_CANCODER_ID);

		kinematics = new SwerveDriveKinematics(
			new Translation2d(Swerve.TRACKWIDTH / 2.0, Swerve.WHEELBASE / 2.0),
			new Translation2d(Swerve.TRACKWIDTH / 2.0, -Swerve.WHEELBASE / 2.0),
			new Translation2d(-Swerve.TRACKWIDTH / 2.0, Swerve.WHEELBASE / 2.0),
			new Translation2d(-Swerve.TRACKWIDTH / 2.0, -Swerve.WHEELBASE / 2.0)
		);
/* 
		leftFrontModule = Mk4iSwerveModuleHelper.createFalcon500Neo(GearRatio.L2, 0, 0, 0, MAX_VELOCITY_METERS_PER_SECOND);
		leftBackModule = Mk4iSwerveModuleHelper.createFalcon500Neo(GearRatio.L2, 0, 0, 0, MAX_VELOCITY_METERS_PER_SECOND);
		rightFrontModule = Mk4iSwerveModuleHelper.createFalcon500Neo(GearRatio.L2, 0, 0, 0, MAX_VELOCITY_METERS_PER_SECOND);
		rightBackModule = Mk4iSwerveModuleHelper.createFalcon500Neo(GearRatio.L2, 0, 0, 0, MAX_VELOCITY_METERS_PER_SECOND);
*/
		
		if (Robot.isReal()) {}

		leftFrontModule = new MkSwerveModuleBuilder()
			.withLayout(
				DRIVEBASE_TAB.getLayout("leftFrontModule", BuiltInLayouts.kGrid)
				.withSize(2, 4)
				.withPosition(4, 0))
			.withGearRatio(Swerve.MK4I_L2_LEFT_FRONT)
			.withDriveMotor(MotorType.FALCON, Swerve.LEFT_FRONT_DRIVE_ID)
			.withSteerMotor(MotorType.NEO, Swerve.LEFT_FRONT_STEER_ID)
			.withSteerEncoderPort(Swerve.LEFT_FRONT_STEER_CANCODER_ID)
			.build();

		leftBackModule = new MkSwerveModuleBuilder()
			.withLayout(
				DRIVEBASE_TAB.getLayout("leftBackModule", BuiltInLayouts.kGrid)
				.withSize(2, 4)
				.withPosition(4, 0))
			.withGearRatio(Swerve.MK4I_L2_LEFT_BACK)
			.withDriveMotor(MotorType.FALCON, Swerve.LEFT_BACK_DRIVE_ID)
			.withSteerMotor(MotorType.NEO, Swerve.LEFT_BACK_STEER_ID)
			.withSteerEncoderPort(Swerve.LEFT_BACK_STEER_CANCODER_ID)
			.build();

		rightFrontModule = new MkSwerveModuleBuilder()
			.withLayout(
				DRIVEBASE_TAB.getLayout("rightFrontModule", BuiltInLayouts.kGrid)
				.withSize(2, 4)
				.withPosition(4, 0))
			.withGearRatio(Swerve.MK4I_L2_RIGHT_FRONT)
			.withDriveMotor(MotorType.FALCON, Swerve.RIGHT_FRONT_DRIVE_ID)
			.withSteerMotor(MotorType.NEO, Swerve.RIGHT_FRONT_STEER_ID)
			.withSteerEncoderPort(Swerve.RIGHT_FRONT_STEER_CANCODER_ID)
			.build();

		rightBackModule = new MkSwerveModuleBuilder()
			.withLayout(
				DRIVEBASE_TAB.getLayout("rightBackModule", BuiltInLayouts.kGrid)
				.withSize(2, 4)
				.withPosition(4, 0))
			.withGearRatio(Swerve.MK4I_L2_RIGHT_BACK)
			.withDriveMotor(MotorType.FALCON, Swerve.RIGHT_BACK_DRIVE_ID)
			.withSteerMotor(MotorType.NEO, Swerve.RIGHT_BACK_STEER_ID)
			.withSteerEncoderPort(Swerve.RIGHT_BACK_STEER_CANCODER_ID)
			.build();


		odometry = new SwerveDriveOdometry(
			kinematics,	Rotation2d.fromDegrees(Robot.getNavX().getAngle()),
			new SwerveModulePosition[] {
				leftFrontModule.getPosition(),
				leftBackModule.getPosition(),
				rightFrontModule.getPosition(),
				rightBackModule.getPosition(),
			}, new Pose2d(Swerve.STARTING_X, Swerve.STARTING_Y, new Rotation2d()));

		poseEstimator = new SwerveDrivePoseEstimator(
			kinematics, 
			new Rotation2d(), 
			new SwerveModulePosition[] {
				new SwerveModulePosition(0.0, new Rotation2d()),
				new SwerveModulePosition(0.0, new Rotation2d()),
				new SwerveModulePosition(0.0, new Rotation2d()),
				new SwerveModulePosition(0.0, new Rotation2d()),
			}, 
			new Pose2d()
		);

	}

	//return instance of swerve subsystem
	public static SwerveSubsystem getSwerve() {
		return instance;
	}

	//return kinematics instance
	public SwerveDriveKinematics getKinematics() {
		return kinematics;
	}

	//return current positition and angle

	public Pose2d getPose() {
		return pose;
	}

	//get average encoders

	public double getEncoderDistance() {
		return (leftFrontEncoder.getPosition() + leftBackEncoder.getPosition() + rightFrontEncoder.getPosition() + rightBackEncoder.getPosition()) / 4.0;
	}

	//reset encoders
	public void setEncoderDistance() {
		leftFrontEncoder.setPosition(0);
		leftBackEncoder.setPosition(0);
		rightFrontEncoder.setPosition(0);
		rightBackEncoder.setPosition(0);
	}

	public void drive(Translation2d translation, double rotation, boolean fieldOriented) {
		rotation *= Swerve.ROTATION_CONSTANT;

		if (fieldOriented && Robot.isReal()) {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, Rotation2d.fromDegrees(Robot.getNavX().getAngle()));
		}
		else if (fieldOriented && !Robot.isReal()) {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, Rotation2d.fromRadians(angle));
		}
		else {
			chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
		}

		drive(kinematics.toSwerveModuleStates(chassisSpeeds));
	}

	public void offCenterDrive(Translation2d translation, double rotation, Translation2d centerOfRotation, boolean fieldOriented) {
		rotation *= Swerve.ROTATION_CONSTANT;

		if (fieldOriented) {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, Rotation2d.fromDegrees(Robot.getNavX().getAngle()));
		}
		else {
			chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
		}

		drive(kinematics.toSwerveModuleStates(chassisSpeeds, centerOfRotation));
	}

	//generic drive method

	public void drive(SwerveModuleState[] state) {

		//map speed of swerve modules to voltage
		leftFrontModule.set(state[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, state[0].angle.getRadians());
		rightFrontModule.set(state[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, state[1].angle.getRadians());
		leftBackModule.set(state[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, state[2].angle.getRadians());
		rightBackModule.set(state[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, state[3].angle.getRadians());
	}

	public void stop() {
		leftFrontModule.set(0, leftFrontModule.getSteerAngle());
		rightFrontModule.set(0, rightBackModule.getSteerAngle());
		leftBackModule.set(0, leftBackModule.getSteerAngle());
		rightBackModule.set(0, rightFrontModule.getSteerAngle());
	}


	@Override
	public void periodic() {
		SmartDashboard.putBoolean("Evaluate", false);

		
		SwerveModuleState[] current = kinematics.toSwerveModuleStates(chassisSpeeds);

		if (Robot.isReal()) {
			//Update Pose
 

			pose = odometry.update(
				Rotation2d.fromDegrees(Robot.getNavX().getAngle()),
				new SwerveModulePosition[] {
					leftFrontModule.getPosition(),
					leftBackModule.getPosition(),
					rightFrontModule.getPosition(),
					rightBackModule.getPosition(),
				}
			);
		}
		else {

			if ((timer.get() * SECOND_TO_MS) - last >= DELAY_MS) {
				last = timer.get();
				SmartDashboard.putBoolean("Evaluate", true);
				angle += chassisSpeeds.omegaRadiansPerSecond / SECOND_TO_MS * DELAY_MS;
				positions[0] += current[0].speedMetersPerSecond / SECOND_TO_MS * DELAY_MS;
				positions[1] += current[1].speedMetersPerSecond / SECOND_TO_MS * DELAY_MS;
				positions[2] += current[2].speedMetersPerSecond / SECOND_TO_MS * DELAY_MS;
				positions[3] += current[3].speedMetersPerSecond / SECOND_TO_MS * DELAY_MS;

				//random 4 lol
				pose = odometry.update(Rotation2d.fromRadians(angle * 4),
				new SwerveModulePosition[] {
					new SwerveModulePosition(positions[0], current[0].angle),
					new SwerveModulePosition(positions[1], current[1].angle),
					new SwerveModulePosition(positions[2], current[2].angle),
					new SwerveModulePosition(positions[3], current[3].angle),
				});

			}
		}

		//Update ShuffleBoard
		DRIVE_ENCODER_ENTRY.setDouble(getEncoderDistance());
		LEFT_FRONT_STEER_ANGLE_ENTRY.setDouble(leftFrontModule.getSteerAngle());
		LEFT_FRONT_STEER_ANGLE_ENTRY.setDouble(leftBackModule.getSteerAngle());
		LEFT_FRONT_STEER_ANGLE_ENTRY.setDouble(rightFrontModule.getSteerAngle());
		LEFT_FRONT_STEER_ANGLE_ENTRY.setDouble(rightBackModule.getSteerAngle());
		NAVX_ANGLE_ENTRY.setDouble(Robot.getNavX().getAngle());
		NAVX_RATE_ENTRY.setDouble(Robot.getNavX().getRate());
		POSITION_X_ENTRY.setDouble(pose.getX());
		POSITION_Y_ENTRY.setDouble(pose.getY());

		//field.setRobotPose(odometry.getPoseMeters());
		Logger.getInstance().recordOutput("Swerve Module States", kinematics.toSwerveModuleStates(chassisSpeeds));
		Logger.getInstance().recordOutput("Odometry", pose);
		

	}
}
