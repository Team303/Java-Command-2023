package com.team303.robot.autonomous;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import static com.team303.robot.autonomous.AutonomousProgram.create;

import com.team303.robot.Robot;
import  com.team303.robot.autonomous.AutonomousProgram;
import com.team303.robot.commands.drive.DriveWait;
import com.team303.robot.commands.drive.FollowTrajectory;
import com.team303.robot.subsystems.PoseEstimatorModule;
import com.team303.robot.subsystems.SwerveSubsystem;

/**
 * Quick guide to Comand Groups:
 *
 * SequentialComandGroup:
 * Will run all comands in order within it's parentheses
 * Note: If a comand does not have a isFinshed statment the code will be stuck
 * on that command forever
 *
 * ParallelCommandGroup:
 * Will run commands in parallel if they use diffrent SubSystems
 * Note: Both commands will have to finish to move on
 *
 * ParallelRaceGoup:
 * Will run commands in parallel if they use diffrent SubSystems
 * Note: As soon as one command runs it's isfinished method runs then both
 * commands will end
 *
 * ParallelDeadlineGroup
 * Will run commands in parallel if they use diffrent SubSystems
 * Note: Only the first command will finish the group
 */
 public class Autonomous {
  	// This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
	// for every path in the group
	static List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(4, 3));
	private static SwerveSubsystem swerve = SwerveSubsystem.getSwerve();
  private static SwerveAutoBuilder autoBuilder;
	// This is just an example event map. It would be better to have a constant, global event map
	// in your code that will be used by all path following commands.
	
	// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.


   public static void init() {
    autoBuilder = new SwerveAutoBuilder(
		swerve::getPose, // Pose2d supplier
		swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
		swerve.getKinematics(), // SwerveDriveKinematics
		new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
		new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
		swerve::drive, // Module states consumer used to output to the drive subsystem
		Robot.eventMap,
		true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
		swerve // The drive subsystem. Used to properly set the requirements of path following commands
	) ;
    /* Start with back against hub */
    
    create (
      "Follow Trajectory",
      () -> {
        try {
          return new SequentialCommandGroup(
            autoBuilder.followPath(pathGroup.get(0))
          );
        } catch (Exception e) {
          e.printStackTrace();
          return null;
        }
      }
    );

    create (
      "Straight Forward",
      () -> {
        try {
          return new SequentialCommandGroup(
            autoBuilder.followPath(pathGroup.get(0))
          );
        } catch (Exception e) {
          e.printStackTrace();
          return null;
        }
      }
    );

    create (
      "DriveWait",
      () -> 
      new SequentialCommandGroup(
        new DriveWait(10)
      )
    );
   }
 }
