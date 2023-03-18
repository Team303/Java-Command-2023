package com.team303.robot.autonomous;

// import com.team303.robot.commands.arm.ReachPoint;
import static com.team303.robot.Robot.CONTROLLER_TAB;
import static com.team303.robot.autonomous.AutonomousProgram.create;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.team303.robot.commands.arm.Homing;
import com.team303.robot.commands.drive.AutoLevelBasic;

import java.util.HashMap;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.team303.robot.RobotMap.Swerve;
import com.team303.robot.Robot;
import static com.team303.robot.Robot.swerve;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.networktables.GenericEntry;

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
        // This will load the file "FullAuto.path" and generate it with a max velocity
        // of 4 m/s and a max acceleration of 3 m/s^2
        // for every path in the group
        // global event map

        // This is just an example event map. It would be better to have a constant,
        // in your code that will be used by all path following commands.

        // Create the AutoBuilder. This only needs to be created once when robot code
        // starts, not every time you want to create an auto command. A good place to
        // put this is in RobotContainer along with your subsystems.

        static List<PathPlannerTrajectory> pathGroup;
        private static SwerveAutoBuilder autoBuilder;
        public static final GenericEntry EFFECTOR_X = CONTROLLER_TAB.add("Set X", 0).getEntry();
        public static final GenericEntry EFFECTOR_Y = CONTROLLER_TAB.add("Set Y", 0).getEntry();

        public static void init() {

                HashMap<String, Command> eventMap = new HashMap<>();
                // //In Inches
                // // eventMap.put("Top Cone", new ReachPoint(-42, 48));
                // // eventMap.put("Middle Cone", new ReachPoint(-24, 35)); //TEST THESE
                // // eventMap.put("Top Cube", new ReachPoint(-42, 36)); //TEST THESE
                // // eventMap.put("Middle Cone", new ReachPoint(-24, 25)); //TEST THESE
                // // eventMap.put("Bottom Hybrid", new ReachPoint(-16, 5)); //TEST THESE
                // // eventMap.put("Reach Cone", new ReachPoint(36, 0));

                autoBuilder = new SwerveAutoBuilder(
                                swerve::getPose, // Pose2d supplier
                                swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the
                                // beginning of auto
                                swerve.getKinematics(), // SwerveDriveKinematics
                                new PIDConstants(5, 0.0, 0.0), // PID constants to correct for translation
                                // error (used to create the X
                                // and Y
                                // PID controllers)
                                new PIDConstants(0.1, 0.0, 0.0), // PID constants to correct for rotation
                                // error (used to create the
                                // rotation
                                // controller)
                                swerve::drive, // Module states consumer used to output to the drive
                                // subsystem
                                eventMap,
                                false, // Should the path be automatically mirrored depending on allianc color.
                                       // Optional, defaults to true
                                swerve // The drive subsystem. Used to properly set the requirements of path
                                       // following
                                       // commands
                );

                // create("New Path", () -> autoBuilder.fullAuto(pathGroup));

                // pathGroup = PathPlanner.loadPathGroup("Top to Cone", new PathConstraints(3,
                // Swerve.MAX_VELOCITY));
                // create("Top to Cone", () -> new SequentialCommandGroup(new
                // InstantCommand(Robot.swerve::resetOdometry),
                // autoBuilder.fullAuto(pathGroup)));
                // pathGroup = PathPlanner.loadPathGroup("Cone Top", new PathConstraints(3,
                // Swerve.MAX_VELOCITY));
                // create("Cone Top", () -> autoBuilder.fullAuto(pathGroup));
                // pathGroup = PathPlanner.loadPathGroup("Cone Middle", new PathConstraints(3,
                // Swerve.MAX_VELOCITY));
                // create("Cone Middle", () -> autoBuilder.fullAuto(pathGroup));
                // pathGroup = PathPlanner.loadPathGroup("Bottom Hybrid", new PathConstraints(3,
                // Swerve.MAX_VELOCITY));
                // create("Bottom Hybrid", () -> autoBuilder.fullAuto(pathGroup));
                // pathGroup = PathPlanner.loadPathGroup("Top Cube", new PathConstraints(3,
                // Swerve.MAX_VELOCITY));
                // create("Top Cube", () -> autoBuilder.fullAuto(pathGroup));
                // pathGroup = PathPlanner.loadPathGroup("Middle Cube", new PathConstraints(3,
                // Swerve.MAX_VELOCITY));
                // create("Middle Cube", () -> autoBuilder.fullAuto(pathGroup));
                pathGroup = PathPlanner.loadPathGroup("Basic Auto", new PathConstraints(3, Swerve.MAX_VELOCITY));
                create("Taxi", () -> new SequentialCommandGroup(autoBuilder.fullAuto(pathGroup), new AutoLevelBasic()));

                // create("New", () -> {
                // try {
                // return new SequentialCommandGroup(
                // new InstantCommand(Robot.getNavX()::reset),
                // new FollowTrajectory("output/New.wpilib.json")
                // );
                // } catch (FileNotFoundException e) {
                // e.printStackTrace();
                // return new DriveWait(15);
                // }});

                // create("StraighForward1", () -> {
                // try {
                // return new SequentialCommandGroup(
                // new InstantCommand(Robot.getNavX()::reset),
                // new FollowTrajectory("output/StraightForward1.wpilib.json")
                // );
                // } catch (FileNotFoundException e) {
                // e.printStackTrace();
                // return new DriveWait(15);
                // }});

                // create("NavX Test", () -> {
                // try {
                // return new SequentialCommandGroup(
                // new InstantCommand(Robot.getNavX()::reset),
                // new TurnToAngle(180),
                // new FollowTrajectory("output/New.wpilib.json")
                // );
                // } catch (FileNotFoundException e) {
                // e.printStackTrace();
                // return new DriveWait(15);
                // }
                // }
                // );

                // create("Drivepose", () -> Robot.swerve.driveToPose(Robot.swerve.getPose(),
                // new Pose2d(5, 5, new Rotation2d()), new Pose2d(4, 4, new Rotation2d())));

                // create("Apriltag", () -> new AlignAprilTag());

                // create("AprilTag", () -> new AprilTagAlign(2));

                // create("Autolevel", () ->
                // new AutolevelFeedforward()
                // );

                // create("Reach Point", () ->
                // new ReachPoint(EFFECTOR_X.getDouble(0.0), EFFECTOR_Y.getDouble(0.0))
                // );

                // create("reach selected", () ->
                // Robot.swerve.driveToPose(Robot.swerve.getPose(), new Pose2d(5, 5, new
                // Rotation2d()), new Pose2d(4, 4, new Rotation2d())));

                // create("drive to node", () -> swerve.driveToNode());

                create("Homing", () -> new Homing());

        }
}
