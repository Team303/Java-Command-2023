package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;
import static com.team303.robot.RobotMap.IOConstants.DEADBAND_FILTER;

import com.team303.robot.Robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultIKControlCommand extends CommandBase {
    public static Translation3d cartesianStorage = new Translation3d(0, 0, 0);
    double x=0;
    double z=0;

    public DefaultIKControlCommand() {
        addRequirements(arm);
    }

    @Override
    public void execute() {
        x = cartesianStorage.getX();
        z = cartesianStorage.getZ();
        cartesianStorage = new Translation3d(
            x+DEADBAND_FILTER.applyDeadband(Robot.getOperatorXbox().getLeftX(), DEADBAND_FILTER.getLowerBound()),
            0.0,
            z-DEADBAND_FILTER.applyDeadband(Robot.getOperatorXbox().getLeftY(), DEADBAND_FILTER.getLowerBound()));
        // System.out.println(cartesianStorage.toString());
        arm.reachEmbedded(cartesianStorage);

    }

}
