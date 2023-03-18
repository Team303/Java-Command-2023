package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;
import static com.team303.robot.RobotMap.IOConstants.DEADBAND_FILTER;

import com.team303.robot.Robot;
import com.team303.robot.RobotMap.Arm;
import com.team303.robot.subsystems.ArmSubsystem;

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
        // angle += Math.toRadians(Robot.getOperatorXbox().getLeftTriggerAxis() - Robot.getOperatorXbox().getRightTriggerAxis());
        length = Math.hypot(cartesianStorage.getZ(), cartesianStorage.getX());

        double robotAngle;
        if (Robot.isReal()) {
            robotAngle = Math.toRadians(-Robot.getNavX().getAngle());
        } else {
            robotAngle = Robot.swerve.angle;
        }

        x = Math.cos(angle) * length;
        z = Math.sin(angle) * length;

        if (fieldOriented) {
            x += (MathUtil.applyDeadband(-Robot.getOperatorXbox().getLeftY(), 0.03) * Math.cos(robotAngle)
                    - MathUtil.applyDeadband(Robot.getOperatorXbox().getLeftX(), 0.03) * Math.sin(robotAngle));
        } else {
            x += DEADBAND_FILTER.applyDeadband(Robot.getOperatorXbox().getLeftX(), DEADBAND_FILTER.getLowerBound());
        }
        z -= DEADBAND_FILTER.applyDeadband(Robot.getOperatorXbox().getRightY(), DEADBAND_FILTER.getLowerBound());

        x = Math.min(x, 48);
        x = Math.max(x, -48);
        z = Math.min(z, 72);
        z = Math.max(z, 0);

        cartesianStorage = new Translation3d(x, 0.0, z);

        arm.reachEmbedded(cartesianStorage);
        arm.armKinematics.updateEmbedded((float) cartesianStorage.getX(), (float) cartesianStorage.getZ());
        effectorRoot.setPosition(
                (Arm.SIMULATION_OFFSET + 150) / Arm.SIMULATION_SCALE + cartesianStorage.getX() / Arm.SIMULATION_SCALE,
                Arm.SIMULATION_OFFSET / Arm.SIMULATION_SCALE + cartesianStorage.getZ() / Arm.SIMULATION_SCALE);
    }
}
