package com.team303.robot.modules;

import com.team303.robot.Robot;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightModule extends SubsystemBase {

    /* ShuffleBoard */
    public static final ShuffleboardTab LIMELIGHT_TAB = Shuffleboard.getTab("limelight");

    public static final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    public static final GenericEntry CROSS_X_TAB = LIMELIGHT_TAB.add("Cross X", 0).getEntry();
    public static final GenericEntry CROSS_Y_TAB = LIMELIGHT_TAB.add("Cross Y", 0).getEntry();
    public static final GenericEntry TARGET_AREA = LIMELIGHT_TAB.add("Target A", 0).getEntry();
    public static final GenericEntry SKEW_AREA = LIMELIGHT_TAB.add("Skew A", 0).getEntry();

    public double getHorizontalCrosshairOffestAngle() {
        return limelight.getEntry("tx").getDouble(0);
    }

    public double getVerticalCrosshairOffsetAngle() {
        return limelight.getEntry("ty").getDouble(0);
    }

    public boolean hasValidTargets() {
        return limelight.getEntry("tv").getDouble(0) > 0;
    }

    public double getTargetArea() {
        return limelight.getEntry("ta").getDouble(0);
    }

    public double getSkewArea() {
        return limelight.getEntry("ts").getDouble(0);
    }

    @Override
    public void periodic() {
        CROSS_X_TAB.setDouble(getHorizontalCrosshairOffestAngle());
        CROSS_Y_TAB.setDouble(getVerticalCrosshairOffsetAngle());
        TARGET_AREA.setDouble(getTargetArea());
        SKEW_AREA.setDouble(getSkewArea());
    }
}