package com.team303.robot.autonomous;

// import com.team303.robot.commands.arm.ReachPoint;
import static com.team303.robot.Robot.CONTROLLER_TAB;
import static com.team303.robot.Robot.swerve;
import static com.team303.robot.autonomous.AutonomousProgram.create;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.team303.robot.RobotMap.Swerve;
import com.team303.robot.commands.arm.ReachPoint;
import com.team303.robot.commands.drive.AutoLevelBasic;
import com.team303.robot.commands.arm.HomeArm;
import com.team303.robot.Robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

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

        static List<PathPlannerTrajectory> pathGroupForward;
        static List<PathPlannerTrajectory> pathGroupSubstation;
        static List<PathPlannerTrajectory> pathGroupGate;
        static List<PathPlannerTrajectory> pathGroupLevel;
        static List<PathPlannerTrajectory> pathGroupLevelScore;
        static List<PathPlannerTrajectory> pathGroupLevelBackwards;
        static List<PathPlannerTrajectory> pathGroupScore;
        private static SwerveAutoBuilder autoBuilder;
        public static final GenericEntry EFFECTOR_X = CONTROLLER_TAB.add("Set X", 0).getEntry();
        public static final GenericEntry EFFECTOR_Y = CONTROLLER_TAB.add("Set Y", 0).getEntry();

        public static void init() {
                HashMap<String, Command> eventMap = new HashMap<>();
                // //In Inches
                // eventMap.put("Top Cone", new SequentialCommandGroup(new ReachPoint(42, 48), new InstantCommand(Robot.claw::toggleState)));
                // eventMap.put("Middle Cone", new SequentialCommandGroup(new ReachPoint(24, 35), new InstantCommand(Robot.claw::toggleState))); //TEST THESE
                // eventMap.put("Top Cube", new SequentialCommandGroup(new ReachPoint(42, 36), new InstantCommand(Robot.claw::toggleState))); //TEST THESE
                // eventMap.put("Middle Cone", new SequentialCommandGroup(new ReachPoint(24, 25), new InstantCommand(Robot.claw::toggleState))); //TEST THESE
                // eventMap.put("Bottom Hybrid", new SequentialCommandGroup(new ReachPoint(16, 5), new InstantCommand(Robot.claw::toggleState))); //TEST THESE
                eventMap.put("Top Cube", new ReachPoint(73, 15));
                eventMap.put("Toggle State", new InstantCommand(Robot.claw::toggleState));

                autoBuilder = new SwerveAutoBuilder(
                                swerve::getPose, // Pose2d supplier
                                swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the
                                // beginning of auto
                                swerve.getKinematics(), // SwerveDriveKinematics
                                new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for translation
                                // error (used to create the X
                                // and Y
                                // PID controllers)
                                new PIDConstants(0, 0, 0.0), // PID constants to correct for rotation
                                // error (used to create the
                                // rotation
                                // controller)
                                swerve::drive, // Module states consumer used to output to the drive
                                // subsystem
                                eventMap,
                                true, // Should the path be automatically mirrored depending on allianc color.
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
                pathGroupLevel = PathPlanner.loadPathGroup("Level", new PathConstraints(3, Swerve.MAX_VELOCITY));
                create("Auto Level", () -> new SequentialCommandGroup(autoBuilder.fullAuto(pathGroupLevel),
                                new AutoLevelBasic()));
                pathGroupSubstation = PathPlanner.loadPathGroup("Towards Substation",
                                new PathConstraints(3, Swerve.MAX_VELOCITY));
                create("Taxi Substation", () -> autoBuilder.fullAuto(pathGroupSubstation));
                pathGroupGate = PathPlanner.loadPathGroup("Towards Gate", new PathConstraints(3, Swerve.MAX_VELOCITY));
                create("Taxi Gate", () -> autoBuilder.fullAuto(pathGroupGate));
                pathGroupForward = PathPlanner.loadPathGroup("Forward", new PathConstraints(3, Swerve.MAX_VELOCITY));
                create("Taxi Forward", () -> autoBuilder.fullAuto(pathGroupForward));
                pathGroupLevelScore = PathPlanner.loadPathGroup("Level and Backup and Score", new PathConstraints(3, Swerve.MAX_VELOCITY));
                create("Forward and Score", () -> new SequentialCommandGroup(autoBuilder.fullAuto(pathGroupLevelScore)));
                create("Score Cube", () -> new ReachPoint(73, 15));
                pathGroupScore = PathPlanner.loadPathGroup("Score", new PathConstraints(3, Swerve.MAX_VELOCITY));

                create("Score Cube and Level", () -> new SequentialCommandGroup(new ReachPoint(73, 15), new ParallelCommandGroup(autoBuilder.fullAuto(pathGroupScore), new HomeArm()), new AutoLevelBasic()));
                // create("Bottom Node", () -> new SequentialCommandGroup(new ReachPoint(16, 5)));
                // create("Middle Node", () -> new ReachPoint(24, 35));
                // create("New", () -> {
                // try {
                // return new SequentialCommandGroup(
                // new InstantCommand(Robot.navX::reset),
                // new FollowTrajectory("output/New.wpilib.json")
                // );
                // } catch (FileNotFoundException e) {
                // e.printStackTrace();
                // return new DriveWait(15);
                // }});

                // create("StraighForward1", () -> {
                // try {
                // return new SequentialCommandGroup(
                // new InstantCommand(Robot.navX::reset),
                // new FollowTrajectory("output/StraightForward1.wpilib.json")
                // );
                // } catch (FileNotFoundException e) {
                // e.printStackTrace();
                // return new DriveWait(15);
                // }});

                // create("NavX Test", () -> {
                // try {
                // return new SequentialCommandGroup(
                // new InstantCommand(Robot.navX::reset),
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

                // create("Reach Point", () -> new ReachPoint(EFFECTOR_X.getDouble(0.0), EFFECTOR_Y.getDouble(0.0)));
                // create("Homing", () -> new HomeArm());

                // create("reach selected", () ->
                // Robot.swerve.driveToPose(Robot.swerve.getPose(), new Pose2d(5, 5, new
                // Rotation2d()), new Pose2d(4, 4, new Rotation2d())));

                // create("drive to node", () -> swerve.driveToNode());
        }
}
