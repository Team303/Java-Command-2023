package frc.robot.autonomous;

import static frc.robot.autonomous.AutonomousProgram.create;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.io.FileNotFoundException;

import frc.robot.commands.drive.DriveWait;
import frc.robot.commands.drive.FollowTrajectory;

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

  public static void init() {
    /* Start with back against hub */
    
    /*create (
      "Follow Trajectory",
      () -> {
        try {
          return new SequentialCommandGroup(
            new FollowTrajectory("PathWeaver/output/Unnamed.wpilib.json")
          );
        } catch (Exception e) {
          e.printStackTrace();
          return null;
        }
      }
    );*/

    create (
      "DriveWait",
      () -> 
      new SequentialCommandGroup(
        new DriveWait(10)
      )
    );
  }
}
