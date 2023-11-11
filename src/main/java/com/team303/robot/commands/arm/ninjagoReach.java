package com.team303.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.team303.robot.util.EffectorState;
import edu.wpi.first.math.geometry.Translation3d;

public class ninjagoReach extends SequentialCommandGroup{

    public ninjagoReach(double xCoordinate, double zCoordinate) {
        super(new ReachPointShoulderElbow(new Translation3d(xCoordinate, 0 , zCoordinate)), 
                new ReachPointWrist(new Translation3d(xCoordinate, 0, zCoordinate)));
    }

}
