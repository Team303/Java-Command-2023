package com.team303.robot.subsystems;


import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import com.team303.robot.Robot;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimatorModule extends SubsystemBase {
    
    private static final SwerveSubsystem swerve = SwerveSubsystem.getSwerve();
    private static final PhotonvisionModule photonvision = PhotonvisionModule.getPhotonvision();
    public final AprilTagFieldLayout aprilTagField;
    public static final ShuffleboardTab tab = Shuffleboard.getTab("Pose Estimation");

    private final Field2d field2d = new Field2d();

    //TODO: Find optimal configuration for each component's influence on Kalman filter
    private static final Vector<N3> swerveStandardDeviations = VecBuilder.fill(0.5,0.5,Units.degreesToRadians(10));
    private static final Vector<N3> photonStandardDeviations = VecBuilder.fill(0.25,0.25,Units.degreesToRadians(5));

    //TODO: Find transformation from camera to robot
    private static final Transform3d CAMERA_TO_ROBOT_TRANSFORM = new Transform3d(new Translation3d(), new Rotation3d());
    private static final Transform3d CAMERA_TO_ARM_TRANSFORM = new Transform3d(new Translation3d(), new Rotation3d());

    public PhotonPoseEstimator visionPoseEstimator;
    public SwerveDrivePoseEstimator poseEstimator;
    public static PoseEstimatorModule instance = new PoseEstimatorModule();

    public PoseEstimatorModule() {
        AprilTagFieldLayout initialLayout;
        try {
            initialLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            var alliance = DriverStation.getAlliance();
            initialLayout.setOrigin(alliance == Alliance.Blue ?
                OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
        } catch(IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            initialLayout = null;
        }
        aprilTagField = initialLayout;
        poseEstimator = new SwerveDrivePoseEstimator(
        swerve.getKinematics(), 
        new Rotation2d(), 
        new SwerveModulePosition[] {
            new SwerveModulePosition(0.0, new Rotation2d()),
            new SwerveModulePosition(0.0, new Rotation2d()),
            new SwerveModulePosition(0.0, new Rotation2d()),
            new SwerveModulePosition(0.0, new Rotation2d()),
        }, 
        new Pose2d(),
        swerveStandardDeviations,
        photonStandardDeviations
        );
        visionPoseEstimator = new PhotonPoseEstimator(aprilTagField, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,photonvision.getCamera(),CAMERA_TO_ROBOT_TRANSFORM.inverse());

        tab.add("Pose", getFomattedPose()).withPosition(0, 0).withSize(2, 0);
        tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
    }
    public static PoseEstimatorModule getPoseSubsystem() {
		return instance;
	}
    public Pose2d getRobotPose() {
        return poseEstimator.getEstimatedPosition();
    }
    //Sets the pose estimation to a new pose
    public void setRobotPose(Pose2d newPose) {
        poseEstimator.resetPosition(
        Rotation2d.fromDegrees(Robot.getNavX().getAngle()),
        swerve.getModulePositions(),
        newPose
        );
    }
    private String getFomattedPose() {
    var pose = getRobotPose();
    return String.format("(%.2f, %.2f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        pose.getRotation().getDegrees());
    }
    public Translation3d getArmtoTargetTranslation() {
		Transform3d camToTarget = photonvision.getBestTarget().getBestCameraToTarget(); 
        Pose3d camPose = new Pose3d(getRobotPose()).transformBy(CAMERA_TO_ROBOT_TRANSFORM.inverse());
        Pose3d armPose = camPose.transformBy(CAMERA_TO_ARM_TRANSFORM);
        return armPose.transformBy(camToTarget).getTranslation();
	}
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        visionPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return visionPoseEstimator.update();
    }
    public static Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
		return new SequentialCommandGroup(
			new InstantCommand(() -> {
			// Reset odometry for the first path you run during auto
			if(isFirstPath){
				swerve.resetOdometry(traj.getInitialHolonomicPose());
			}
			}),
			new PPSwerveControllerCommand(
				traj, 
				getPoseSubsystem()::getRobotPose, // Pose supplier
				swerve.getKinematics(), // SwerveDriveKinematics
				new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
				new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
				new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
				swerve::drive, // Module states consumer
				true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
				swerve 
			)
		); 
	} 

    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> result = getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
        if (result.isPresent()) {
            EstimatedRobotPose visionPoseEstimate = result.get();
            poseEstimator.addVisionMeasurement(visionPoseEstimate.estimatedPose.toPose2d(), visionPoseEstimate.timestampSeconds);
        }
        poseEstimator.update(
        Rotation2d.fromDegrees(Robot.getNavX().getAngle()),
        swerve.getModulePositions()
        );
        field2d.setRobotPose(getRobotPose());

    }
}
