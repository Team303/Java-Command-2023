// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team303.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.team303.lib.kinematics.FabrikController;
import com.team303.robot.Robot;
import com.team303.robot.commands.arm.DefaultIKControlCommand;
import com.team303.robot.RobotMap.Arm;
import com.team303.robot.commands.arm.DefaultIKControlCommand;

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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.GenericEntry;
import static com.team303.robot.commands.arm.DefaultIKControlCommand.cartesianStorage;
import com.revrobotics.CANSparkMax.IdleMode;

public class ArmSubsystem extends SubsystemBase {

	private final double GEAR_RATIO = 40 / 12 * 160;

	public static final ShuffleboardTab ARM_TAB = Shuffleboard.getTab("Arm");
	public static final NetworkTable armNetwork = NetworkTableInstance.getDefault().getTable("arm");
	public static final GenericEntry shoulderAngle = ARM_TAB.add("shoulderAngle", 0).getEntry();
	public static final GenericEntry jointAngle = ARM_TAB.add("jointAngle", 0).getEntry();
	public static final GenericEntry clawAngle = ARM_TAB.add("clawAngle", 0).getEntry();
	public static final GenericEntry effectorX = ARM_TAB.add("effectorX", 0).getEntry();
	public static final GenericEntry effectorY = ARM_TAB.add("effectorY", 0).getEntry();
    public static final GenericEntry shoulderAngleAbsolute = ARM_TAB.add("shoulderAngleAbsolute", 0).getEntry();
	public static final GenericEntry jointAngleAbsolute = ARM_TAB.add("jointAngleAbsolute", 0).getEntry();
	public static final GenericEntry clawAngleAbsolute = ARM_TAB.add("clawAngleAbsolute", 0).getEntry();
    public static final GenericEntry effectorAngle = ARM_TAB.add("effector angle", 0).getEntry();

	public static final DoubleArraySubscriber JOINT_ANGLES_SUB = armNetwork.getDoubleArrayTopic("ja")
			.subscribe(new double[] { 0.0, 0.0, 0.0 });
	public static final DoubleArraySubscriber JOINT_RPM_SUB = armNetwork.getDoubleArrayTopic("jr")
			.subscribe(new double[] { 0.0, 0.0, 0.0 });

	public static final DoubleArrayPublisher JOINT_ANGLES_PUB = armNetwork.getDoubleArrayTopic("Joint Angles Out")
			.publish();
	public static final DoubleArrayPublisher JOINT_RPM_PUB = armNetwork.getDoubleArrayTopic("Joints RPM Out").publish();

	public class ShoulderJoint {
		private final CANSparkMax shoulderMotor1 = new CANSparkMax(Arm.SHOULDER_JOINT_ID1, MotorType.kBrushless);
		private final CANSparkMax shoulderMotor2 = new CANSparkMax(Arm.SHOULDER_JOINT_ID2, MotorType.kBrushless);

		public ShoulderJoint() {
			shoulderMotor1.setIdleMode(IdleMode.kBrake);
			shoulderMotor2.setIdleMode(IdleMode.kBrake);

			shoulderMotor1.setInverted(false);
			shoulderMotor2.setInverted(true);
		}

		private final void setMotors(double speed) {
			shoulderMotor1.set(speed);
			shoulderMotor2.set(speed);
		}

		private final RelativeEncoder shoulderEncoder = shoulderMotor1.getEncoder();
		public final ProfiledPIDController shoulderControl = new ProfiledPIDController(0.01, 0, 0,
				new TrapezoidProfile.Constraints(Units.rotationsToRadians(62) / 60, 100));
		private final ArmFeedforward m_shoulderFeedForward = new ArmFeedforward(0.01, 0, 0, 0);
		private MechanismLigament2d shoulderSimulator;

	}

	public class ElbowJoint {
		private final CANSparkMax elbowMotor = null; // new CANSparkMax(Arm.ELBOW_JOINT_ID, MotorType.kBrushless);
		private final RelativeEncoder elbowEncoder = null; //elbowMotor.getEncoder();
		public ProfiledPIDController elbowControl = new ProfiledPIDController(0.01, 0, 0,
				new TrapezoidProfile.Constraints(Units.rotationsToRadians(62) / 60, 100));
		private final ArmFeedforward m_elbowFeedForward = new ArmFeedforward(0.01, 0, 0, 0);
		private MechanismLigament2d elbowSimulator;

	}

	public class ClawJoint {
		private final CANSparkMax clawMotor = null; //new CANSparkMax(Arm.CLAW_JOINT_ID, MotorType.kBrushless);
		private final RelativeEncoder clawEncoder = null; //clawMotor.getEncoder();
		public ProfiledPIDController clawControl = new ProfiledPIDController(0.01, 0, 0,
				new TrapezoidProfile.Constraints(Units.rotationsToRadians(62) / 60, 100));
		private final ArmFeedforward m_clawFeedForward = new ArmFeedforward(0.01, 0, 0, 0);
		private MechanismLigament2d clawSimulator;

	}

	public static FabrikController armKinematics = new FabrikController();
	
	private ShoulderJoint shoulderJoint = new ShoulderJoint();
	private ElbowJoint elbowJoint = new ElbowJoint();
	private ClawJoint clawJoint = new ClawJoint();
	
	private Mechanism2d armSimulation;
	public MechanismRoot2d effectorRoot;
	private MechanismLigament2d effectorPoint;
	
	private double storedShoulderAngle; 
	private double storedElbowAngle;
	private double storedClawAngle ;

	public ArmSubsystem() {
		// Initialize Inverse Kinematics with constant values
		armKinematics.setArmLength(84f);
		armKinematics.setSegmentLengthRatio(0, 36 / 84f);
		armKinematics.setSegmentLengthRatio(1, 36 / 84f);
		armKinematics.setSegmentLengthRatio(2, 12 / 84f);
		armKinematics.setSegmentLengths();
		// armKinematics.setAngleConstraint(0, 360, 360);
		// armKinematics.setAngleConstraint(1, 360, 360);
		// armKinematics.setAngleConstraint(2, 360, 360);
		armKinematics.setAngleConstraint(0, 90, 20);
		armKinematics.setAngleConstraint(1, 60, 180);
		armKinematics.setAngleConstraint(2, 45, 135);
		armKinematics.setSegmentInitialDirection(0, (float) Math.toRadians(90));
		armKinematics.setSegmentInitialDirection(1, (float) Math.toRadians(0));
		armKinematics.setSegmentInitialDirection(2, (float) Math.toRadians(-45));
		armKinematics.initializeArm();
		armKinematics.setSolveDistanceThreshold(1f);
		armKinematics.setMaxIterationAttempts(5000);
		// TODO: Find joint gear ratios
		shoulderJoint.shoulderEncoder.setPositionConversionFactor(1);
		// 60 motor rotations = 360 degrees of rotation for the arm
		// elbowJoint.elbowEncoder.setPositionConversionFactor(60);
		// clawJoint.clawEncoder.setPositionConversionFactor(1);
		armSimulation = new Mechanism2d(300, 300);
		MechanismRoot2d armRoot = armSimulation.getRoot("Arm", Arm.SIMULATION_OFFSET, Arm.SIMULATION_OFFSET);
		shoulderJoint.shoulderSimulator = armRoot.append(new MechanismLigament2d("shoulder",
				(double) armKinematics.getSegmentLength(0), 0.0, 5.0, new Color8Bit(255, 0, 0)));
		elbowJoint.elbowSimulator = shoulderJoint.shoulderSimulator.append(new MechanismLigament2d("elbow",
				(double) armKinematics.getSegmentLength(1), 0.0, 5.0, new Color8Bit(0, 255, 0)));
		clawJoint.clawSimulator = elbowJoint.elbowSimulator.append(new MechanismLigament2d("claw",
				(double) armKinematics.getSegmentLength(2), 0.0, 5.0, new Color8Bit(0, 0, 255)));
		//effectorRoot = armSimulation.getRoot("End Effector", Arm.SIMULATION_OFFSET+armKinematics.getEffectorPoint().get(0),Arm.SIMULATION_OFFSET+armKinematics.getEffectorPoint().get(1));
		effectorRoot = armSimulation.getRoot("End Effector", Arm.SIMULATION_OFFSET+cartesianStorage.getX(),Arm.SIMULATION_OFFSET+cartesianStorage.getZ());

		effectorPoint = effectorRoot.append(new MechanismLigament2d("effector",1.0,0.0,5.0,new Color8Bit(125,125,125)));
		storedShoulderAngle = -90+armKinematics.getIKAnglesDegrees().get(0);
		storedElbowAngle = storedShoulderAngle-armKinematics.getIKAnglesDegrees().get(1);
		storedClawAngle = storedElbowAngle+armKinematics.getIKAnglesDegrees().get(2);
	}
	public static NetworkTable getArmNetwork() {
		if (armNetwork == null) {
			return NetworkTableInstance.getDefault().getTable("arm");
		}
		return armNetwork;
	}

	public void reach(Translation3d translation) {
		armKinematics.solveTargetIK(translation);
		// Units of error are inches
		reach(armKinematics.getIKAnglesRadians());
	}

	public void reachEmbedded(Translation3d translation) {
		armKinematics.solveForEmbedded();

        reach(armKinematics.getIKAnglesRadians());
	}


	public void reach(List<Double> desiredRadianAngles) {
		for (double i : desiredRadianAngles) {
			System.out.print(Math.toDegrees(i)+" ");
		}
		shoulderAngle.setDouble(Math.toDegrees(desiredRadianAngles.get(0)));
		jointAngle.setDouble(Math.toDegrees(desiredRadianAngles.get(1)));
		clawAngle.setDouble(Math.toDegrees(desiredRadianAngles.get(2)));
		System.out.println();

		double shoulderEncoders = (desiredRadianAngles.get(0) / Math.PI * 2) * 42 * GEAR_RATIO;

		shoulderJoint.shoulderControl.setGoal(shoulderEncoders);
		elbowJoint.elbowControl.setGoal(desiredRadianAngles.get(1));
		clawJoint.clawControl.setGoal(desiredRadianAngles.get(2));
		double shoulderFeedForward = shoulderJoint.m_shoulderFeedForward.calculate(
				shoulderJoint.shoulderControl.getGoal().position, shoulderJoint.shoulderControl.getGoal().velocity);
		// double elbowFeedForward = elbowJoint.m_elbowFeedForward.calculate(elbowJoint.elbowControl.getGoal().position,
		// 		elbowJoint.elbowControl.getGoal().velocity);
		// double clawFeedForward = clawJoint.m_clawFeedForward.calculate(clawJoint.clawControl.getGoal().position,
		// 		clawJoint.clawControl.getGoal().velocity);

		double shoulderFeedback = shoulderJoint.shoulderControl.calculate(
				shoulderJoint.shoulderEncoder.getPosition(),
				shoulderJoint.shoulderControl.getGoal());
		// double elbowFeedback = elbowJoint.elbowControl.calculate(
		// 		Units.rotationsToRadians(elbowJoint.elbowEncoder.getPosition()), elbowJoint.elbowControl.getGoal());
		// double clawFeedback = clawJoint.clawControl.calculate(
		// 		Units.rotationsToRadians(clawJoint.clawEncoder.getPosition()), clawJoint.clawControl.getGoal());

		SmartDashboard.putNumber("shoulderFeedForward", shoulderFeedForward);
		SmartDashboard.putNumber("shoulderFeeback", shoulderFeedback);

		shoulderJoint.setMotors(shoulderFeedForward + shoulderFeedback);
		// elbowJoint.elbowMotor.set(elbowFeedForward + elbowFeedback);
		// clawJoint.clawMotor.set(clawFeedForward + clawFeedback);
	}

	public void resetEncoders() {
		shoulderJoint.shoulderEncoder.setPosition(0.0);
		// elbowJoint.elbowEncoder.setPosition(0.0);
		// clawJoint.clawEncoder.setPosition(0.0);
	}

	// public double[] getEncoderPosition() {
	// 	return new double[] { shoulderJoint.shoulderEncoder.getPosition(), elbowJoint.elbowEncoder.getPosition(),
	// 			clawJoint.clawEncoder.getPosition() };
	// }

	// public double[] getJointResolutions() {
	// 	return new double[] { shoulderJoint.shoulderEncoder.getCountsPerRevolution(),
	// 			elbowJoint.elbowEncoder.getCountsPerRevolution(), clawJoint.clawEncoder.getCountsPerRevolution() };
	// }

	public float getError() {
		return armKinematics.getIKPositionError();
	}

	@Override
	public void periodic() {
		JOINT_ANGLES_PUB.set(JOINT_ANGLES_SUB.get(new double[] { 0.0, 0.0, 0.0 }));
		JOINT_RPM_PUB.set(JOINT_RPM_SUB.get(new double[] { 0.0, 0.0, 0.0 }));
		double shoulderSimAngle = 90-armKinematics.getIKAnglesDegrees().get(0);
		double elbowSimAngle =-armKinematics.getIKAnglesDegrees().get(1);
		double clawSimAngle = -armKinematics.getIKAnglesDegrees().get(2);

		if (shoulderSimAngle != storedShoulderAngle || elbowSimAngle != storedElbowAngle || clawSimAngle != storedClawAngle) {
		//System.out.println("Shoulder Angle: "+shoulderSimAngle+" Elbow Angle: "+elbowSimAngle+" Claw Angle: "+clawSimAngle);
		storedShoulderAngle = shoulderSimAngle;
		storedElbowAngle = elbowSimAngle;
		storedClawAngle = clawSimAngle;
        double absoluteElbow = storedShoulderAngle + storedElbowAngle;
        double absoluteClaw = storedClawAngle + storedShoulderAngle + storedElbowAngle;
        double targetAngle = Math.toDegrees(Math.atan2(cartesianStorage.getZ(), cartesianStorage.getX()));
        
        // if (targetAngle < storedShoulderAngle) {
        //     storedElbowAngle = -storedElbowAngle;
        // }
        // if (targetAngle < absoluteElbow) {
        //     storedClawAngle = -storedClawAngle;
        // }

        shoulderAngleAbsolute.setDouble(storedShoulderAngle);
        jointAngleAbsolute.setDouble(absoluteElbow);
        clawAngleAbsolute.setDouble(absoluteClaw);
        
		}
		// System.out.println(cartesianStorage.toString());	
		shoulderJoint.shoulderSimulator.setAngle(storedShoulderAngle);
		elbowJoint.elbowSimulator.setAngle(storedElbowAngle);
		clawJoint.clawSimulator.setAngle(storedClawAngle);
        effectorAngle.setDouble(Math.toDegrees(Math.atan2(cartesianStorage.getZ(), cartesianStorage.getX())));
		effectorX.setDouble(cartesianStorage.getX());
		effectorY.setDouble(cartesianStorage.getZ());
		Logger.getInstance().recordOutput("MyMechanism", this.armSimulation);
	} 
}
