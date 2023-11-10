package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;
import static com.team303.robot.commands.arm.DefaultIKControlCommand.cartesianStorage;

import com.team303.robot.Robot;
import com.team303.robot.RobotMap.Arm;
import com.team303.robot.subsystems.ArmSubsystem;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.List;
import com.team303.robot.util.EffectorState;

public class ReachPoint extends CommandBase {
    public Translation3d cartesianCoords;
    private final double TOLERANCE = 6;
    private EffectorState effectorState;

    List<Double> angles;

    public ReachPoint(double x, double z, EffectorState effectorState) {
        this.effectorState = effectorState;
        this.cartesianCoords = new Translation3d(x, 0.0, z);
        addRequirements(arm);
    }

    public ReachPoint(Translation3d cartesianCoords, EffectorState effectorState) {
        this.effectorState = effectorState;
        this.cartesianCoords = cartesianCoords;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        angles = arm.reachEmbedded(cartesianCoords, effectorState);
        Robot.arm.effectorRoot.setPosition(
                (Arm.SIMULATION_OFFSET + 150) / Arm.SIMULATION_SCALE + cartesianCoords.getX(),
                Arm.SIMULATION_OFFSET / Arm.SIMULATION_OFFSET + cartesianStorage.getZ());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Math.toDegrees(arm.shoulderJoint.getJointAngle() - angles.get(0))) < TOLERANCE &&
                Math.abs(Math.toDegrees(arm.elbowJoint.getJointAngle() - angles.get(1))) < TOLERANCE &&
                Math.abs(Math.toDegrees(arm.wristJoint.getJointAngle() - angles.get(2))) < TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        cartesianStorage = new Translation3d(cartesianCoords.getX(), 0, cartesianCoords.getZ());
        ArmSubsystem.getXCoordinateTab().setDouble(cartesianCoords.getX());
        ArmSubsystem.getYCoordinateTab().setDouble(cartesianCoords.getZ());
        
    }

}
