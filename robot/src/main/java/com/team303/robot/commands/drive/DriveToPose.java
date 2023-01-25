package com.team303.robot.commands.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPoint;
import com.team303.robot.subsystems.PoseEstimatorModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToPose extends CommandBase {
    private PathPlannerTrajectory trajectory;
    public DriveToPose(Pose2d startPose, Pose2d endPose) {
            trajectory = PathPlanner.generatePath(
            new PathConstraints(4, 3), 
            new PathPoint(startPose.getTranslation(), startPose.getRotation()), // position, heading
            new PathPoint(endPose.getTranslation(), endPose.getRotation()) // position, heading
            );
    }

    @Override
    public void execute() {
        PoseEstimatorModule.followTrajectoryCommand(trajectory, false);
    }
    
}
