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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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
	public class ShoulderJoint {
		private final CANSparkMax shoulderMotor = new CANSparkMax(Arm.SHOULDER_JOINT_ID, MotorType.kBrushless);
		private final RelativeEncoder shoulderEncoder = shoulderMotor.getEncoder();
		public final ProfiledPIDController shoulderControl = new ProfiledPIDController(0.01,0,0,new TrapezoidProfile.Constraints(Units.rotationsToRadians(62)/60,100));
		private final ArmFeedforward m_shoulderFeedForward = new ArmFeedforward(0.01,0,0,0);

	}
	public class ElbowJoint {
		private final CANSparkMax elbowMotor = new CANSparkMax(Arm.ELBOW_JOINT_ID, MotorType.kBrushless);
		private final RelativeEncoder elbowEncoder = elbowMotor.getEncoder();
		public ProfiledPIDController elbowControl = new ProfiledPIDController(0.01,0,0,new TrapezoidProfile.Constraints(Units.rotationsToRadians(62)/60,100));
		private final ArmFeedforward m_elbowFeedForward = new ArmFeedforward(0.01,0,0,0);
	} 
	public class ClawJoint {
		private final CANSparkMax clawMotor = new CANSparkMax(Arm.CLAW_JOINT_ID, MotorType.kBrushless);
		private final RelativeEncoder clawEncoder = clawMotor.getEncoder();
		public ProfiledPIDController clawControl = new ProfiledPIDController(0.01,0,0,new TrapezoidProfile.Constraints(Units.rotationsToRadians(62)/60,100));
		private final ArmFeedforward m_clawFeedForward = new ArmFeedforward(0.01,0,0,0);
	}
	private static ArmSubsystem instance = new ArmSubsystem();
	private static IKWrapper caliko = new IKWrapper();
	private ShoulderJoint shoulderJoint;
	private ElbowJoint elbowJoint;
	private ClawJoint clawJoint;
	private ArmSubsystem() {
		//Initialize Caliko with constant values
		caliko.setArmLength(84f);
		caliko.setSegmentLengthRatio(0,35/84f);
		caliko.setSegmentLengthRatio(1,35/84f);
		caliko.setSegmentLengthRatio(2,12/84f);
		caliko.setSegmentLengths();
		caliko.setAngleConstraint(0,45,45);
		caliko.setAngleConstraint(1,135,135);
		caliko.setAngleConstraint(2,135,135);
		caliko.setSegmentInitialDirection(0,(float)Math.PI/2);
		caliko.setSegmentInitialDirection(1,0f);
		caliko.setSegmentInitialDirection(2,(float)-Math.PI/4);
		caliko.initializeArm();
		shoulderJoint = new ShoulderJoint();
		elbowJoint = new ElbowJoint();
		clawJoint = new ClawJoint();

		//TODO: Find joint gear ratios
		shoulderJoint.shoulderEncoder.setPositionConversionFactor(1);
		//60 motor rotations = 360 degrees of rotation for the arm
		elbowJoint.elbowEncoder.setPositionConversionFactor(60);
		clawJoint.clawEncoder.setPositionConversionFactor(1);
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
	public void reach(Translation3d translation) {
		caliko.solveTargetIK(translation);
		//Units of error are inches
		if (caliko.getIKPositionError()>=5) {
			System.out.println("Exception: Target out of range.");
			return;
		}
		reach(caliko.getIKAnglesRadians());
	}
	public void reach(double[]desiredRadianAngles) {
		shoulderJoint.shoulderControl.setGoal(desiredRadianAngles[0]);
		elbowJoint.elbowControl.setGoal(desiredRadianAngles[1]);
		clawJoint.clawControl.setGoal(desiredRadianAngles[2]);
		double shoulderFeedForward = shoulderJoint.m_shoulderFeedForward.calculate(shoulderJoint.shoulderControl.getGoal().position,shoulderJoint.shoulderControl.getGoal().velocity);
		double elbowFeedForward = elbowJoint.m_elbowFeedForward.calculate(elbowJoint.elbowControl.getGoal().position,elbowJoint.elbowControl.getGoal().velocity);
		double clawFeedForward = clawJoint.m_clawFeedForward.calculate(clawJoint.clawControl.getGoal().position,clawJoint.clawControl.getGoal().velocity);

		double shoulderFeedback = shoulderJoint.shoulderControl.calculate(Units.rotationsToRadians(shoulderJoint.shoulderEncoder.getPosition()),shoulderJoint.shoulderControl.getGoal());
		double elbowFeedback = elbowJoint.elbowControl.calculate(Units.rotationsToRadians(elbowJoint.elbowEncoder.getPosition()),elbowJoint.elbowControl.getGoal());
		double clawFeedback = clawJoint.clawControl.calculate(Units.rotationsToRadians(clawJoint.clawEncoder.getPosition()),clawJoint.clawControl.getGoal());

		shoulderJoint.shoulderMotor.setVoltage(shoulderFeedForward+shoulderFeedback);
		elbowJoint.elbowMotor.setVoltage(elbowFeedForward+elbowFeedback);
		clawJoint.clawMotor.setVoltage(clawFeedForward+clawFeedback);
	}

	public void resetEncoders() {
		shoulderJoint.shoulderEncoder.setPosition(0.0);
		elbowJoint.elbowEncoder.setPosition(0.0);
		clawJoint.clawEncoder.setPosition(0.0);	
	}
	public double[] getEncoderPosition() {
		return new double[]{shoulderJoint.shoulderEncoder.getPosition(),elbowJoint.elbowEncoder.getPosition(),clawJoint.clawEncoder.getPosition()};
	}
	public double[] getJointResolutions() {
		return new double[]{shoulderJoint.shoulderEncoder.getCountsPerRevolution(),elbowJoint.elbowEncoder.getCountsPerRevolution(),clawJoint.clawEncoder.getCountsPerRevolution()};
	}


	@Override
	public void periodic() {
		JOINT_ANGLES_PUB.set(JOINT_ANGLES_SUB.get(new double[]{0.0,0.0,0.0}));
        JOINT_RPM_PUB.set(JOINT_RPM_SUB.get(new double[]{0.0,0.0,0.0}));
	}
}
