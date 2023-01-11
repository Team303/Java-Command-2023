// Copyright (c) 2022 Team 303

package frc.robot.subsystems;

import javax.swing.plaf.basic.BasicSplitPaneUI.KeyboardHomeHandler;

import com.ctre.phoenix.sensors.CANCoder;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio;
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
import com.revrobotics.CANSparkMax;
import frc.robot.Robot;
import frc.robot.RobotMap.Swerve;

public class SwerveSubsystem extends SubsystemBase {

	/* ShuffleBoard */
	public static final ShuffleboardTab DRIVEBASE_TAB = Shuffleboard.getTab("Drivebase");

	public static final GenericEntry NAVX_ANGLE_ENTRY = DRIVEBASE_TAB.add("NavX Angle", 0).getEntry();
	public static final GenericEntry NAVX_RATE_ENTRY = DRIVEBASE_TAB.add("NavX Rate", 0).getEntry();
	public static final GenericEntry LEFT_FRONT_STEER_ANGLE_ENTRY = DRIVEBASE_TAB.add("Front Left Steer Angle", 0).getEntry();
	public static final GenericEntry RIGHT_FRONT_STEER_ANGLE_ENTRY = DRIVEBASE_TAB.add("Front Right Steer Angle", 0).getEntry();
	public static final GenericEntry LEFT_BACK_STEER_ANGLE_ENTRY = DRIVEBASE_TAB.add("Back Left Steer Angle", 0).getEntry();
	public static final GenericEntry RIGHT_BACK_STEER_ANGLE_ENTRY = DRIVEBASE_TAB.add("Back Right Steer Angle", 0).getEntry();
	public static final GenericEntry DRIVE_ENCODER_ENTRY = DRIVEBASE_TAB.add("Average Encoders", 0).getEntry();

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

	/*Shuffleboard Tab*/
	private static final ShuffleboardTab SWERVE_TAB = Shuffleboard.getTab("Swerve");

	private Pose2d pose;

	public static final double MAX_VOLTAGE = 12.0;

	public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
		SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
		SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

	private static SwerveSubsystem instance;

	public SwerveSubsystem() {

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

		leftFrontModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
			SWERVE_TAB.getLayout("leftFrontModule"),
			GearRatio.L2,
			Swerve.LEFT_FRONT_DRIVE_ID,
			Swerve.LEFT_FRONT_STEER_ID,
			Swerve.LEFT_FRONT_STEER_CANCODER_ID,
			Swerve.LEFT_FRONT_STEER_OFFSET
		);

		leftBackModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
			SWERVE_TAB.getLayout("leftBackModule"),
			GearRatio.L2,
			Swerve.LEFT_BACK_DRIVE_ID,
			Swerve.LEFT_BACK_STEER_ID,
			Swerve.LEFT_BACK_STEER_CANCODER_ID,
			Swerve.LEFT_BACK_STEER_OFFSET
		);

		rightFrontModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
			SWERVE_TAB.getLayout("rightFrontModule"),
			GearRatio.L2,
			Swerve.RIGHT_FRONT_DRIVE_ID,
			Swerve.RIGHT_FRONT_STEER_ID,
			Swerve.RIGHT_FRONT_STEER_CANCODER_ID,
			Swerve.RIGHT_FRONT_STEER_OFFSET
		);

		rightBackModule = Mk4iSwerveModuleHelper.createFalcon500Neo(
			SWERVE_TAB.getLayout("rightBackModule"),
			GearRatio.L2,
			Swerve.RIGHT_BACK_DRIVE_ID,
			Swerve.RIGHT_BACK_STEER_ID,
			Swerve.RIGHT_BACK_STEER_CANCODER_ID,
			Swerve.RIGHT_BACK_STEER_OFFSET
		);



		odometry = new SwerveDriveOdometry(
			kinematics,	Rotation2d.fromDegrees(Robot.getNavX().getAngle()),
			new SwerveModulePosition[] {
				new SwerveModulePosition(leftFrontModule.getDriveDistance(), Rotation2d.fromRadians(leftFrontModule.getSteerAngle())),
				new SwerveModulePosition(leftBackModule.getDriveDistance(), Rotation2d.fromRadians(leftBackModule.getSteerAngle())),
				new SwerveModulePosition(rightFrontModule.getDriveDistance(), Rotation2d.fromRadians(rightFrontModule.getSteerAngle())),
				new SwerveModulePosition(rightBackModule.getDriveDistance(), Rotation2d.fromRadians(rightBackModule.getSteerAngle()))
			}, new Pose2d(Swerve.STARTING_X, Swerve.STARTING_Y, new Rotation2d()));
	}

	/* return instance of swerve subsystem*/
	public static SwerveSubsystem getSwerve() {

		if (instance == null) {
			return new SwerveSubsystem();
		}
		return instance;
	}

	/* return kinematics instance */
	public SwerveDriveKinematics getKinematics() {
		return kinematics;
	}

	/* return current positition and angle */

	public Pose2d getPose() {
		return pose;
	}

	/*get average encoders*/

	public double getEncoderDistance() {
		return (leftFrontEncoder.getPosition() + leftBackEncoder.getPosition() + rightFrontEncoder.getPosition() + rightBackEncoder.getPosition()) / 4.0;
	}

	/*reset encoders*/
	public void setEncoderDistance() {
		leftFrontEncoder.setPosition(0);
		leftBackEncoder.setPosition(0);
		rightFrontEncoder.setPosition(0);
		rightBackEncoder.setPosition(0);
	}

	public void drive(Translation2d translation, double rotation, boolean fieldOriented) {
		rotation *= Swerve.ROTATION_CONSTANT;

		if (fieldOriented) {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, Rotation2d.fromDegrees(Robot.getNavX().getAngle()));
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

	/*generic drive method*/

	public void drive(SwerveModuleState[] state) {

		/*map speed of swerve modules to voltage*/
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
			/* Update ShuffleBoard */
			DRIVE_ENCODER_ENTRY.setDouble(getEncoderDistance());
			LEFT_FRONT_STEER_ANGLE_ENTRY.setDouble(leftFrontModule.getSteerAngle());
			LEFT_FRONT_STEER_ANGLE_ENTRY.setDouble(leftBackModule.getSteerAngle());
			LEFT_FRONT_STEER_ANGLE_ENTRY.setDouble(rightFrontModule.getSteerAngle());
			LEFT_FRONT_STEER_ANGLE_ENTRY.setDouble(rightBackModule.getSteerAngle());
			NAVX_ANGLE_ENTRY.setDouble(Robot.getNavX().getAngle());
			NAVX_RATE_ENTRY.setDouble(Robot.getNavX().getRate());

			/* Update Pose */
			Rotation2d angle = Rotation2d.fromDegrees(Robot.getNavX().getAngle());

			pose = odometry.update(angle,
			new SwerveModulePosition[] {
				new SwerveModulePosition(leftFrontModule.getDriveDistance(), Rotation2d.fromRadians(leftFrontModule.getSteerAngle())),
				new SwerveModulePosition(leftBackModule.getDriveDistance(), Rotation2d.fromRadians(leftBackModule.getSteerAngle())),
				new SwerveModulePosition(rightFrontModule.getDriveDistance(), Rotation2d.fromRadians(rightFrontModule.getSteerAngle())),
				new SwerveModulePosition(rightBackModule.getDriveDistance(), Rotation2d.fromRadians(rightBackModule.getSteerAngle()))
			});
	}
}
