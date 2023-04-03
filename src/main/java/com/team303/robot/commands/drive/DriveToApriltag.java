package com.team303.robot.commands.drive;

import com.team303.robot.Robot;
import com.team303.robot.RobotMap.LimelightConstants;
import com.team303.robot.modules.LimelightModule.CameraName;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToApriltag extends CommandBase {

    private final PIDController FeedbackController = new PIDController(0.025, 0, 0.01);
    double leftDistance;
    double rightDistance;
    double averageDistance;

    public DriveToApriltag() {
        FeedbackController.enableContinuousInput(-1, 1);
        FeedbackController.setTolerance(2, 0);
        addRequirements(Robot.swerve);
    }

    @Override
    public void execute() {
        double leftID = Robot.limelight.getAprilTagID(CameraName.CAM1);
        double rightID = Robot.limelight.getAprilTagID(CameraName.CAM2);
        if (leftID == rightID && (leftID != 4 || rightID != 5)) {
            leftDistance = Robot.limelight.estimateDistance(LimelightConstants.CAMERA_PITCH_RADIANS,
                    LimelightConstants.CAMERA_HEIGHT_METERS, LimelightConstants.GRID_TARGET_HEIGHT_METERS,
                    CameraName.CAM1);
            rightDistance = Robot.limelight.estimateDistance(LimelightConstants.CAMERA_PITCH_RADIANS,
                    LimelightConstants.CAMERA_HEIGHT_METERS, LimelightConstants.GRID_TARGET_HEIGHT_METERS,
                    CameraName.CAM2);
            averageDistance = leftDistance + rightDistance / 2;
            Robot.swerve.drive(
                    new Translation2d(0, FeedbackController.calculate(averageDistance, 0)),
                    0, true);
        } else if (leftID == rightID) {
            leftDistance = Robot.limelight.estimateDistance(LimelightConstants.CAMERA_PITCH_RADIANS,
                    LimelightConstants.CAMERA_HEIGHT_METERS, LimelightConstants.DOUBLE_SUBSTATION_TARGET_HEIGHT_METERS,
                    CameraName.CAM1);
            rightDistance = Robot.limelight.estimateDistance(LimelightConstants.CAMERA_PITCH_RADIANS,
                    LimelightConstants.CAMERA_HEIGHT_METERS, LimelightConstants.DOUBLE_SUBSTATION_TARGET_HEIGHT_METERS,
                    CameraName.CAM2);
            averageDistance = leftDistance + rightDistance / 2;
            Robot.swerve.drive(
                    new Translation2d(0, FeedbackController.calculate(averageDistance, 0)),
                    0, true);
        } else if (leftID == 4 || rightID == 5) {
            leftDistance = Robot.limelight.estimateDistance(LimelightConstants.CAMERA_PITCH_RADIANS,
                    LimelightConstants.CAMERA_HEIGHT_METERS, LimelightConstants.DOUBLE_SUBSTATION_TARGET_HEIGHT_METERS,
                    CameraName.CAM1);
            averageDistance = leftDistance;
            Robot.swerve.drive(
                    new Translation2d(0, FeedbackController.calculate(averageDistance, 0)),
                    0, true);
        } else {
            leftDistance = Robot.limelight.estimateDistance(LimelightConstants.CAMERA_PITCH_RADIANS,
                    LimelightConstants.CAMERA_HEIGHT_METERS, LimelightConstants.GRID_TARGET_HEIGHT_METERS,
                    CameraName.CAM1);
            averageDistance = leftDistance;
            Robot.swerve.drive(
                    new Translation2d(0, FeedbackController.calculate(averageDistance, 0)),
                    0, true);

        }

    }

    @Override
    public boolean isFinished() {
        return averageDistance <= 0.5; // Change the num
    }
}
