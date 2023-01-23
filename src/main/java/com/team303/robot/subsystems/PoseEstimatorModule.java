package com.team303.robot.subsystems;


import java.io.IOException;
import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.team303.robot.Robot;
import com.team303.robot.subsystems.PhotonvisionModule.PhotonPipeline;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimatorModule extends SubsystemBase {
    
    private static final SwerveSubsystem swerve = SwerveSubsystem.getSwerve();
    private static final PhotonvisionModule photonvision = PhotonvisionModule.getPhotonvision();
    private final AprilTagFieldLayout aprilTagField;
    public static final ShuffleboardTab tab = Shuffleboard.getTab("Pose Estimation");

    private final Field2d field2d = new Field2d();

    //TODO: Find optimal configuration for each component's influence on Kalman filter
    private static final Vector<N3> swerveStandardDeviations = VecBuilder.fill(0.5,0.5,Units.degreesToRadians(10));
    private static final Vector<N3> photonStandardDeviations = VecBuilder.fill(0.25,0.25,Units.degreesToRadians(5));

    private double previousPipelineTimestamp = 0;
    //TODO: Find linear transformatoin from camera to robot
    private static final Transform3d CAMERA_TO_ROBOT_TRANSFORM = new Transform3d(new Translation3d(), new Rotation3d());
    public SwerveDrivePoseEstimator poseEstimator;

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

        tab.add("Pose", getFomattedPose()).withPosition(0, 0).withSize(2, 0);
        tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
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

    @Override
    public void periodic() {
        PhotonPipelineResult pipelineResult = photonvision.getCamera().getLatestResult();
        double resultTimestamp = pipelineResult.getTimestampSeconds();
        if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets() && photonvision.getPipeline() == PhotonPipeline.APRILTAG) {
            previousPipelineTimestamp = resultTimestamp;
            PhotonTrackedTarget target = pipelineResult.getBestTarget();
            int fiducialId = target.getFiducialId();
            // Get the tag pose from field layout - consider that the layout will be null if it failed to load
            Optional<Pose3d> tagPose = aprilTagField == null ? Optional.empty() : aprilTagField.getTagPose(fiducialId);
            if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
                Pose3d targetPose = tagPose.get();
                Transform3d camToTarget = target.getBestCameraToTarget();
                Pose3d camPose = targetPose.transformBy(camToTarget.inverse());
                Pose3d visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT_TRANSFORM);
                poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
            }
        }
        poseEstimator.update(
        Rotation2d.fromDegrees(Robot.getNavX().getAngle()),
        swerve.getModulePositions()
        );
        field2d.setRobotPose(getRobotPose());

    }
}
