package com.team303.robot.subsystems;

import edu.wpi.first.wpilibj.event.NetworkBooleanEvent;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;

public class LimelightModule extends SubsystemBase {

    /* ShuffleBoard */
	public static final ShuffleboardTab DRIVEBASE_TAB = Shuffleboard.getTab("Drivebase");
    
    public static final GenericEntry VALID_TARGETS = DRIVEBASE_TAB.add("Valid Targets", 0).getEntry();
    public static final GenericEntry OFFSET_X = DRIVEBASE_TAB.add("Offset X", 0).getEntry();
    public static final GenericEntry OFFSET_Y = DRIVEBASE_TAB.add("Offset Y", 0).getEntry();
    public static final GenericEntry TARGET_AREA = DRIVEBASE_TAB.add("Target Area", 0).getEntry();
    public static final GenericEntry SKEW_ROTATION = DRIVEBASE_TAB.add("Skew Rotation", 0).getEntry();

    public static final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight"); 

    public static NetworkTable getLimelight() {
        if (limelight == null) {
            return NetworkTableInstance.getDefault().getTable("limelight");
        }
        return limelight;
    }

    @Override
    public void periodic() {
        VALID_TARGETS.setDouble(limelight.getEntry("tv").getDouble(0));
        OFFSET_X.setDouble(limelight.getEntry("tx").getDouble(0));
        OFFSET_Y.setDouble(limelight.getEntry("ty").getDouble(0));
        TARGET_AREA.setDouble(limelight.getEntry("ta").getDouble(0));
        SKEW_ROTATION.setDouble(limelight.getEntry("ts").getDouble(0));
    }
}

