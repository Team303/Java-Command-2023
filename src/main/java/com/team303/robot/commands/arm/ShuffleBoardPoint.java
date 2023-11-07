package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;
import static com.team303.robot.commands.arm.DefaultIKControlCommand.cartesianStorage;
import com.team303.robot.subsystems.ArmSubsystem;
import com.team303.robot.Robot;
import com.team303.robot.RobotMap.Arm;
import com.team303.robot.subsystems.ArmSubsystem;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.List;

public class ShuffleBoardPoint extends CommandBase {

    public ShuffleBoardPoint() {
        addRequirements(arm);
        //cartesianStorage = new Translation3d(x, 0, z);
        //ArmSubsystem.lloydkaijayzanecolenyamasterwudarethninjagorulesaryasucksaravruleshappybirthdayarav();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.lloydkaijayzanecolenyamasterwudarethninjagorulesaryasucksaravruleshappybirthdayarav();
    }
}
