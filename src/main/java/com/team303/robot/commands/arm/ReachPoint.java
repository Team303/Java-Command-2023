package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;
import static com.team303.robot.RobotMap.IOConstants.DEADBAND_FILTER;
import com.team303.robot.subsystems.ArmSubsystem;
import com.team303.robot.Robot;
import com.team303.robot.RobotMap.Arm;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReachPoint extends CommandBase {
    public Translation3d cartesianStorage;

    public ReachPoint(double x, double z) {
        this.cartesianStorage = new Translation3d(x, 0, z);
        addRequirements(arm);
    }

    public ReachPoint(Translation3d cartesianStorage) {
        this.cartesianStorage = cartesianStorage;
        addRequirements(arm);
    }

    @Override
    public void execute() {
    
        // System.out.println(cartesianStorage.toString());
        arm.reachEmbedded(cartesianStorage);
        ArmSubsystem.armKinematics.updateEmbedded((float) cartesianStorage.getX(), (float) cartesianStorage.getZ());
        Robot.arm.effectorRoot.setPosition(Arm.SIMULATION_OFFSET+cartesianStorage.getX(),Arm.SIMULATION_OFFSET+cartesianStorage.getZ());
    }

    // @Override
    // public boolean isFinished() {
    //     return Math.abs(arm.getError()) < 3.0;
    // }

}
