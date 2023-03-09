package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;
import static com.team303.robot.RobotMap.IOConstants.DEADBAND_FILTER;
import com.team303.robot.RobotMap.Arm;
import com.team303.robot.Robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team303.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import com.team303.robot.RobotMap.Arm;

public class DefaultIKControlCommand extends CommandBase {
    public static Translation3d cartesianStorage = new Translation3d(0, 0, 0);
    double x = cartesianStorage.getX();
    double z = cartesianStorage.getZ();
    double angle = Math.atan2(x, z);
    double length = Math.hypot(x, z);
    MechanismRoot2d effectorRoot;

    public DefaultIKControlCommand() {
        addRequirements(arm);
        this.effectorRoot = Robot.arm.effectorRoot;
    }

    @Override
    public void execute() {
        angle = Math.atan2(cartesianStorage.getZ(), cartesianStorage.getX());
        angle += Math.toRadians(Robot.getOperatorXbox().getLeftTriggerAxis() - Robot.getOperatorXbox().getRightTriggerAxis());
        length = Math.hypot(cartesianStorage.getZ(), cartesianStorage.getX());
        x = Math.cos(angle) * length;
        z = Math.sin(angle) * length;

        
        cartesianStorage = new Translation3d(
            x+DEADBAND_FILTER.applyDeadband(Robot.getOperatorXbox().getLeftX(), DEADBAND_FILTER.getLowerBound()),
            0.0,
            z-DEADBAND_FILTER.applyDeadband(Robot.getOperatorXbox().getLeftY(), DEADBAND_FILTER.getLowerBound()));
        // System.out.println(cartesianStorage.toString());
        arm.reachEmbedded(cartesianStorage);
        ArmSubsystem.armKinematics.updateEmbedded((float) cartesianStorage.getX(), (float) cartesianStorage.getZ());
        effectorRoot.setPosition(Arm.SIMULATION_OFFSET+cartesianStorage.getX(),Arm.SIMULATION_OFFSET+cartesianStorage.getZ());
    }
}
