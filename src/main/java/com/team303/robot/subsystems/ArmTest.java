package com.team303.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.GenericEntry;
import com.team303.robot.Robot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

public class ArmTest extends SubsystemBase {

    CANSparkMax motor1;
    CANSparkMax motor2;
    CANSparkMax elbowMotor;
    CANSparkMax wristMotor;
    RelativeEncoder shoulderEncoder;
    RelativeEncoder elbowEncoder;
    RelativeEncoder wristEncoder;
    SparkMaxLimitSwitch motor1Switch;
    SparkMaxLimitSwitch motor2Switch;
    SparkMaxLimitSwitch elbowSwitch;
    SparkMaxLimitSwitch motor1SwitchForward;
    SparkMaxLimitSwitch motor2SwitchForward;
    SparkMaxLimitSwitch elbowSwitchForward;

    public static final ShuffleboardTab ARM_TEST_TAB = Shuffleboard.getTab("Arm Test");
    GenericEntry shoulderEncodersTab = ARM_TEST_TAB.add("shoulder", 0).getEntry();
    GenericEntry elbowEncodersTab = ARM_TEST_TAB.add("elbow", 0).getEntry();
    GenericEntry wristEncodersTab = ARM_TEST_TAB.add("wrist", 0).getEntry();
    GenericEntry shoulder1SwitchTab = ARM_TEST_TAB.add("shoulder1 switch", false).getEntry();
    GenericEntry shoulder2SwitchTab = ARM_TEST_TAB.add("shoulder2 switch", false).getEntry();
    GenericEntry elbowSwitchTab = ARM_TEST_TAB.add("elbow switch", false).getEntry();
    GenericEntry shoulder1SwitchForwardTab = ARM_TEST_TAB.add("shoulder1 switch forward ", false).getEntry();
    GenericEntry shoulder2SwitchForwardTab = ARM_TEST_TAB.add("shoulder2 switch forward", false).getEntry();
    GenericEntry elbowSwitchForwardTab = ARM_TEST_TAB.add("elbow switch forward", false).getEntry();
    GenericEntry countsPerRev = ARM_TEST_TAB.add("counts per rev", 0).getEntry();
    Timer timer;
    double start;

    public ArmTest() {

        timer = new Timer();
        timer.start();
        start = timer.get();

        motor1 = new CANSparkMax(15, MotorType.kBrushless);
        motor2 = new CANSparkMax(14, MotorType.kBrushless);

        elbowMotor = new CANSparkMax(16, MotorType.kBrushless);
        wristMotor = new CANSparkMax(17, MotorType.kBrushless);

        motor1.setIdleMode(IdleMode.kBrake);
        motor2.setIdleMode(IdleMode.kBrake);
        elbowMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setIdleMode(IdleMode.kBrake);
        
        motor1.setInverted(false);
        motor2.setInverted(true);
        elbowMotor.setInverted(false);
        wristMotor.setInverted(true);

        shoulderEncoder = motor1.getEncoder();
        elbowEncoder = elbowMotor.getEncoder();
        wristEncoder = wristMotor.getEncoder();

        shoulderEncoder.setPositionConversionFactor(266.66);
        elbowEncoder.setPositionConversionFactor(125);
        wristEncoder.setPositionConversionFactor(45);

        // shoulderEncoder.setInverted(false);
        // elbowEncoder.setInverted(false);
        // wristEncoder.setInverted(false);

        motor1Switch = motor1.getReverseLimitSwitch(Type.kNormallyOpen);
        motor2Switch = motor2.getReverseLimitSwitch(Type.kNormallyOpen);
        elbowSwitch = elbowMotor.getReverseLimitSwitch(Type.kNormallyOpen);

        motor1SwitchForward = motor1.getForwardLimitSwitch(Type.kNormallyOpen);
        motor2SwitchForward = motor2.getForwardLimitSwitch(Type.kNormallyOpen);
        elbowSwitchForward = elbowMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    }

    public void move(double speed, double elbowSpeed, double wristSpeed) {
        motor1.set(speed);
        motor2.set(speed);
        elbowMotor.set(elbowSpeed * 0.2);
        wristMotor.set(wristSpeed * 0.2);
    }


    @Override
    public void periodic() {
        // if ((timer.get() - start) >= 100) {
            shoulderEncodersTab.setDouble(shoulderEncoder.getPosition());
            elbowEncodersTab.setDouble(elbowEncoder.getPosition());
            wristEncodersTab.setDouble(wristEncoder.getPosition());
            shoulder1SwitchTab.setBoolean(motor1Switch.isPressed());
            shoulder2SwitchTab.setBoolean(motor2Switch.isPressed());
            elbowSwitchTab.setBoolean(elbowSwitch.isPressed());
            shoulder1SwitchForwardTab.setBoolean(motor1SwitchForward.isPressed());
            shoulder2SwitchForwardTab.setBoolean(motor2SwitchForward.isPressed());
            elbowSwitchForwardTab.setBoolean(elbowSwitchForward.isPressed()); 
            countsPerRev.setDouble(shoulderEncoder.getCountsPerRevolution());
            SmartDashboard.putNumber("encoders per rev Neo", shoulderEncoder.getCountsPerRevolution());
            // System.out.println("Encoders rev neo: " + shoulderEncoder.getCountsPerRevolution());
        //     start = timer.get();
        // }

    }
}
