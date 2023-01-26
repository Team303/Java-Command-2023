package com.team303.robot.modules;

import com.team303.robot.Robot;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    /* ShuffleBoard */
    public static final ShuffleboardTab LIMELIGHT_TAB = Shuffleboard.getTab("limelight");

    public static final NetworkTable limelight = Robot.getNetworkTableInstance().getTable("limelight");

    public static final DoubleSubscriber VALID_TARGETS_SUB = limelight.getDoubleTopic("tv").subscribe(0);
    public static final DoubleSubscriber OFFSET_X_SUB = limelight.getDoubleTopic("tx").subscribe(0.0);
    public static final DoubleSubscriber OFFSET_Y_SUB = limelight.getDoubleTopic("ty").subscribe(0.0);
    public static final DoubleSubscriber TARGET_AREA_SUB = limelight.getDoubleTopic("ta").subscribe(0.0);
    public static final DoubleSubscriber SKEW_ROTATION_SUB = limelight.getDoubleTopic("ts").subscribe(0.0);

    public static final DoublePublisher VALID_TARGETS_PUB = limelight.getDoubleTopic("Valid Targets Out").publish();
    public static final DoublePublisher OFFSET_X_PUB = limelight.getDoubleTopic("Offset X Out").publish();
    public static final DoublePublisher OFFSET_Y_PUB = limelight.getDoubleTopic("Offset Y Out").publish();
    public static final DoublePublisher TARGET_AREA_PUB = limelight.getDoubleTopic("Target Area Out").publish();
    public static final DoublePublisher SKEW_ROTATION_PUB = limelight.getDoubleTopic("Skew Rotation Out").publish();

    public static NetworkTable getLimelight() {
        return limelight;
    }

    public double getHorizontalCrosshairOffestAngle() {
        return limelight.getEntry("tx").getDouble(0);
    }

    public double getVerticalCrosshairOfssetAngle() {
        return limelight.getEntry("ty").getDouble(0);
    }

    public boolean hasValidTargets() {
        return limelight.getEntry("tv").getDouble(0) > 0;
    }

    public double getTargetArea() {
        return limelight.getEntry("ta").getDouble(0);
    }

    @Override
    public void periodic() {
        VALID_TARGETS_PUB.set(VALID_TARGETS_SUB.get(0.0));
        OFFSET_X_PUB.set(OFFSET_X_SUB.get(0.0));
        OFFSET_Y_PUB.set(OFFSET_Y_SUB.get(0.0));
        TARGET_AREA_PUB.set(TARGET_AREA_SUB.get(0.0));
        SKEW_ROTATION_PUB.set(SKEW_ROTATION_SUB.get(0.0));
    }
}
