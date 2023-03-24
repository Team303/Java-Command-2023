package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;
import static com.team303.robot.RobotMap.IOConstants.DEADBAND_FILTER;

import com.team303.robot.Robot;
import com.team303.robot.RobotMap.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultIKControlCommand extends CommandBase {
    public static Translation3d cartesianStorage = new Translation3d(30, 0, 20);
    double x = cartesianStorage.getX();
    double z = cartesianStorage.getZ();
    double angle = Math.atan2(x, z);
    double length = Math.hypot(x, z);
    MechanismRoot2d effectorRoot;
    boolean fieldOriented;

    public DefaultIKControlCommand(boolean fieldOriented) {
        addRequirements(arm);
        this.effectorRoot = Robot.arm.effectorRoot;
        this.fieldOriented = fieldOriented;
    }

    @Override
    public void execute() {
        angle = Math.atan2(cartesianStorage.getZ(), cartesianStorage.getX());
        // angle += Math.toRadians(Robot.operatorController.getLeftTriggerAxis() -
        // Robot.operatorController.getRightTriggerAxis());
        length = Math.hypot(cartesianStorage.getZ(), cartesianStorage.getX());

        double robotAngle;
        if (Robot.isReal()) {
            robotAngle = Math.toRadians(-Robot.navX.getAngle());
        } else {
            robotAngle = Robot.swerve.angle;
        }

        x = Math.cos(angle) * length;
        z = Math.sin(angle) * length;

        if (fieldOriented) {
            x += (MathUtil.applyDeadband(-Robot.operatorController.getLeftY(), 0.03) * Math.cos(robotAngle)
                    - MathUtil.applyDeadband(Robot.operatorController.getLeftX(), 0.03) * Math.sin(robotAngle));
            z -= DEADBAND_FILTER.applyDeadband(Robot.operatorController.getRightY(), DEADBAND_FILTER.getLowerBound());
        } else {
            x += DEADBAND_FILTER.applyDeadband(Robot.operatorController.getLeftX(), DEADBAND_FILTER.getLowerBound());
            z -= DEADBAND_FILTER.applyDeadband(Robot.operatorController.getLeftY(), DEADBAND_FILTER.getLowerBound());
        }
        x = Math.min(x, 48 + 36);
        x = Math.max(x, -48 - 36);
        z = Math.min(z, 72);
        z = Math.max(z, 0);

        cartesianStorage = new Translation3d(x, 0.0, z);
        
        arm.reachEmbedded(cartesianStorage);

        effectorRoot.setPosition(
                (Arm.SIMULATION_OFFSET + 150) / Arm.SIMULATION_SCALE + cartesianStorage.getX() / Arm.SIMULATION_SCALE,
                Arm.SIMULATION_OFFSET / Arm.SIMULATION_SCALE + cartesianStorage.getZ() / Arm.SIMULATION_SCALE);
    }
}
