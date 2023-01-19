// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team303.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.team303.robot.Robot;
import com.team303.robot.RobotMap.Arm;
import com.team303libs.kinematics.IKWrapper;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.team303.lib.kinematics.IKWrapper;

public class ArmSubsystem extends SubsystemBase {

	public static final ShuffleboardTab ARM_TAB = Shuffleboard.getTab("Arm");
	public static final NetworkTable armNetwork = Robot.getNetworkTableInstance().getTable("arm");
	public static final DoubleArraySubscriber JOINT_ANGLES_SUB = armNetwork.getDoubleArrayTopic("ja").subscribe(new double[]{0.0,0.0,0.0});
	public static final DoubleArraySubscriber JOINT_RPM_SUB = armNetwork.getDoubleArrayTopic("jr").subscribe(new double[]{0.0,0.0,0.0});

	public static final DoubleArrayPublisher JOINT_ANGLES_PUB = armNetwork.getDoubleArrayTopic("Joint Angles Out").publish();
	public static final DoubleArrayPublisher JOINT_RPM_PUB = armNetwork.getDoubleArrayTopic("Joints RPM Out").publish();
	/* Joint motors */
	private final CANSparkMax shoulderJoint;
	private final CANSparkMax elbowJoint;
	private final CANSparkMax clawJoint;
	/* Joint encoders */
	private final RelativeEncoder shoulderEncoder;
	private final RelativeEncoder elbowEncoder;
	private final RelativeEncoder clawEncoder;
	private static ArmSubsystem instance = new ArmSubsystem();
	private ArmSubsystem() {
		//Initialize Caliko with constant values
		IKWrapper arm = new IKWrapper();
		arm.setArmLength(84f);
		arm.setSegmentLengthRatio(0,35/84f);
		arm.setSegmentLengthRatio(1,35/84f);
		arm.setSegmentLengthRatio(2,12/84f);
		arm.setSegmentLengths();
		arm.setAngleConstraint(0,45,45);
		arm.setAngleConstraint(1,135,135);
		arm.setAngleConstraint(2,135,135);
		arm.setSegmentInitialDirection(0,(float)Math.PI/2);
		arm.setSegmentInitialDirection(1,0f);
		arm.setSegmentInitialDirection(2,(float)-Math.PI/4);
		arm.initializeArm();

		shoulderJoint = new CANSparkMax(Arm.SHOULDER_JOINT_ID, MotorType.kBrushless);
		elbowJoint = new CANSparkMax(Arm.ELBOW_JOINT_ID, MotorType.kBrushless);
		clawJoint = new CANSparkMax(Arm.CLAW_JOINT_ID, MotorType.kBrushless);
		shoulderEncoder = shoulderJoint.getEncoder();
		elbowEncoder = elbowJoint.getEncoder();
		clawEncoder = clawJoint.getEncoder();
	}
	public static NetworkTable getArmNetwork() {
        if (armNetwork == null) {
            return NetworkTableInstance.getDefault().getTable("arm");
        }
        return armNetwork;
    }
	public static ArmSubsystem getArm() {
		return instance;
	}
	public void resetEncoders() {
		shoulderEncoder.setPosition(0.0);
		elbowEncoder.setPosition(0.0);
		clawEncoder.setPosition(0.0);	
	}
	public double[] encoderPosition() {
		return new double[]{shoulderEncoder.getPosition(),elbowEncoder.getPosition(),clawEncoder.getPosition()};
	}
	public double[] getJointRPMs() {
		return new double[]{shoulderEncoder.getCountsPerRevolution(),elbowEncoder.getCountsPerRevolution(),clawEncoder.getCountsPerRevolution()};
	}


	@Override
	public void periodic() {
		JOINT_ANGLES_PUB.set(JOINT_ANGLES_SUB.get(new double[]{0.0,0.0,0.0}));
        JOINT_RPM_PUB.set(JOINT_RPM_SUB.get(new double[]{0.0,0.0,0.0}));
	}
}
