package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;
import static com.team303.robot.commands.arm.DefaultIKControlCommand.cartesianStorage;

import com.team303.robot.Robot;
import com.team303.robot.RobotMap.Arm;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.List;

public class ReachPointContinuous extends CommandBase {
    public Translation3d cartesianCoords;
    List<Double> angles;

    public ReachPointContinuous(double x, double z) {
        this.cartesianCoords = new Translation3d(x, 0.0, z);
        addRequirements(arm);
    }

    public ReachPointContinuous(Translation3d cartesianCoords) {
        this.cartesianCoords = cartesianCoords;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        angles = arm.reachEmbedded(cartesianCoords);
        System.out.println("running reachpoint");
        Robot.arm.effectorRoot.setPosition(
                (Arm.SIMULATION_OFFSET + 150) / Arm.SIMULATION_SCALE + cartesianCoords.getX(),
                Arm.SIMULATION_OFFSET / Arm.SIMULATION_OFFSET + cartesianStorage.getZ());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Math.toDegrees(arm.shoulderJoint.leftEncoder.getPosition() - angles.get(0))) < 2 &&
            Math.abs(Math.toDegrees(arm.elbowJoint.encoder.getPosition() - angles.get(1))) < 2 &&
            Math.abs(Math.toDegrees(arm.wristJoint.encoder.getPosition() - angles.get(2))) < 2;
    }

    @Override
    public void end(boolean interrupted) {
        cartesianStorage = new Translation3d(cartesianCoords.getX(), 0, cartesianCoords.getZ());
    }

}
