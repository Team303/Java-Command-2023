package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;
import java.util.Arrays;
import java.util.List;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReachAngles extends CommandBase {
    List<Double> desiredAngles;
    public ReachAngles(double shoulder, double elbow, double wrist) {
        addRequirements(arm);
        desiredAngles = Arrays.asList(shoulder ,elbow , wrist);
    }

    @Override
    public void execute() {
        arm.reach(desiredAngles);
    }
}