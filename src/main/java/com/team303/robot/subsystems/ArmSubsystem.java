// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team303.robot.subsystems;

import static com.team303.robot.commands.arm.DefaultIKControlCommand.cartesianStorage;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.team303.lib.kinematics.FabrikController;
import com.team303.lib.kinematics.ArmChain;
import com.team303.robot.RobotMap.Arm;
import com.team303.robot.Robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

	public static final ShuffleboardTab ARM_TAB = Shuffleboard.getTab("Arm");
	public static final NetworkTable armNetwork = NetworkTableInstance.getDefault().getTable("arm");

	/* Row 1 */

	public static final GenericEntry shoulderAngleEntry = ARM_TAB.add("Shoulder Angle", 0).withPosition(0, 0)
			.getEntry();
	public static final GenericEntry elbowAngleEntry = ARM_TAB.add("Elbow Angle", 0).withPosition(1, 0).getEntry();
	public static final GenericEntry wristAngleEntry = ARM_TAB.add("Wrist Angle", 0).withPosition(2, 0).getEntry();

	public static final GenericEntry effectorX = ARM_TAB.add("effectorX", 0).withPosition(3, 0).getEntry();
	public static final GenericEntry effectorY = ARM_TAB.add("effectorY", 0).withPosition(4, 0).getEntry();

	/* Row 2 */

	public static final GenericEntry shoulderEncoderEntrySim = ARM_TAB.add("Should Encoder Sim", 0).withPosition(0, 1)
			.getEntry();
	public static final GenericEntry elbowEncoderEntrySim = ARM_TAB.add("Elbow Encoder Sim", 0).withPosition(1, 1)
			.getEntry();
	public static final GenericEntry wristEncoderEntrySim = ARM_TAB.add("Wrist Encoder Sim", 0).withPosition(2, 1)
			.getEntry();

	/* Row 3 */

	public static final GenericEntry shoulderEncoderEntry = ARM_TAB.add("Should Encoder", 0).withPosition(0, 2)
			.getEntry();
	public static final GenericEntry elbowEncoderEntry = ARM_TAB.add("Elbow Encoder", 0).withPosition(1, 2).getEntry();
	public static final GenericEntry wristEncoderEntry = ARM_TAB.add("Wrist Encoder", 0).withPosition(2, 2).getEntry();

	// public static final GenericEntry clawEncoderTab = ARM_TAB.add("clawEnc",
	// 0).getEntry();s
	// public static final GenericEntry shoulderAngleAbsolute =
	// ARM_TAB.add("shoulderAngleAbsolute", 0).getEntry();
	// public static final GenericEntry jointAngleAbsolute =
	// ARM_TAB.add("jointAngleAbsolute", 0).getEntry();
	// public static final GenericEntry clawAngleAbsoludte =
	// ARM_TAB.add("clawAngleAbsolute", 0).getEntry();
	// public static final GenericEntry effectorAngle = ARM_TAB.add("effector
	// angle", 0).getEntry();
	public static final GenericEntry shoulderSwitchReverseEntry = ARM_TAB.add("shoulder switch reverse", false)
			.withPosition(2, 1).getEntry();
	public static final GenericEntry elbowSwitchForwardTab = ARM_TAB.add("elbow switch forward", false)
			.withPosition(3, 3).getEntry();
	public static final GenericEntry wristSwitchReverseTab = ARM_TAB.add("wrist switch reverse", false).withPosition(5,5).getEntry();

	public static final GenericEntry shoulderSpeed = ARM_TAB.add("shoulder speed", 0).withPosition(3, 4).getEntry();
	public static final GenericEntry elbowSpeed = ARM_TAB.add("elbow speed", 0).withPosition(2, 4).getEntry();
	public static final GenericEntry elbowGoalPos = ARM_TAB.add("elbow goal pos", 0).withPosition(0, 3).getEntry();
	public static final GenericEntry elbowGoalVel = ARM_TAB.add("elbow goal vel", 0).withPosition(1, 3).getEntry();
	public static final GenericEntry shoulderGoalPos = ARM_TAB.add("shoulder goal pos", 0).withPosition(0, 4)
			.getEntry();
	public static final GenericEntry shoulderGoalVel = ARM_TAB.add("shoulder goal Vel", 0).withPosition(1, 4)
			.getEntry();

	public static final double INCHES_TO_METERS = 0.0254;
	public static final double ARMLEN_METERS = INCHES_TO_METERS * 74;
	//public static final double CIRCUMFERENCE = 2 * Math.PI * ARMLEN_METERS;

	// rotations/sec
	public static final double VELOCITY = 2;

	// rotations/sec^2
	public static final double ACCELERATION = 2;

	public static final DoubleArraySubscriber JOINT_ANGLES_SUB = armNetwork.getDoubleArrayTopic("ja")
			.subscribe(new double[] { 0.0, 0.0, 0.0 });
	public static final DoubleArraySubscriber JOINT_RPM_SUB = armNetwork.getDoubleArrayTopic("jr")
			.subscribe(new double[] { 0.0, 0.0, 0.0 });

	public static final DoubleArrayPublisher JOINT_ANGLES_PUB = armNetwork.getDoubleArrayTopic("Joint Angles Out")
			.publish();
	public static final DoubleArrayPublisher JOINT_RPM_PUB = armNetwork.getDoubleArrayTopic("Joints RPM Out").publish();

	public double degreesToEncoders(double angle, double gearRatio) {
		return angle / 360 * 4096 * gearRatio;
	}

	public double radiansToEncoders(double angle, double gearRatio) {
		return angle / (Math.PI * 2) * 4096 * gearRatio;
	}

	public double encodersToRadians(double encoders, double gearRatio) {
		return encoders / gearRatio / 4096 * (Math.PI * 2);
	}

	public class ShoulderJoint {
		private final CANSparkMax leftMotor = new CANSparkMax(Arm.SHOULDER_JOINT_LEFT_ID, MotorType.kBrushless);
		private final CANSparkMax rightMotor = new CANSparkMax(Arm.SHOULDER_JOINT_RIGHT_ID, MotorType.kBrushless);

		public final RelativeEncoder leftEncoder = leftMotor.getEncoder();
		public final RelativeEncoder rightEncoder = rightMotor.getEncoder();

		public final SparkMaxLimitSwitch switchReverse;

		public ShoulderJoint() {
			leftMotor.setIdleMode(IdleMode.kBrake);
			rightMotor.setIdleMode(IdleMode.kBrake);

			switchReverse = rightMotor.getReverseLimitSwitch(Type.kNormallyOpen);

			leftMotor.setInverted(true);
			rightMotor.setInverted(false);

			leftEncoder.setPositionConversionFactor(360 * (1 / Arm.GEAR_RATIO_SHOULDER));
			rightEncoder.setPositionConversionFactor(360 * (1 / Arm.GEAR_RATIO_SHOULDER));
		}

		public final void setMotors(double speed) {
			leftMotor.set(speed);
			rightMotor.set(speed);
		}

		public final ProfiledPIDController controller = new ProfiledPIDController(0.7, 0, 0,
				new TrapezoidProfile.Constraints(VELOCITY, ACCELERATION));

		private final ArmFeedforward feedForward = new ArmFeedforward(0.01, 0, 0, 0);
		private MechanismLigament2d simulator;

	}

	public class ElbowJoint {
		public final CANSparkMax motor = new CANSparkMax(Arm.ELBOW_JOINT_ID, MotorType.kBrushless);
		public final RelativeEncoder encoder = motor.getEncoder();

		public final SparkMaxLimitSwitch switchReverse;
		public final SparkMaxLimitSwitch switchForward;

		public ElbowJoint() {
			switchReverse = motor.getReverseLimitSwitch(Type.kNormallyOpen);
			switchForward = motor.getForwardLimitSwitch(Type.kNormallyOpen);

			motor.setIdleMode(IdleMode.kBrake);
			motor.setInverted(false);

			encoder.setPositionConversionFactor(360 * (1 / Arm.GEAR_RATIO_ELBOW));
		}

		public ProfiledPIDController controller = new ProfiledPIDController(0.7, 0.05, 0,
				new TrapezoidProfile.Constraints(VELOCITY, ACCELERATION));
		private final ArmFeedforward feedForward = new ArmFeedforward(0, -0.4, 0, 0);
		private MechanismLigament2d simulator;
	}

	public class WristJoint {
		public final CANSparkMax motor = new CANSparkMax(Arm.CLAW_JOINT_ID, MotorType.kBrushless);
		private final RelativeEncoder encoder = motor.getEncoder();

		public final SparkMaxLimitSwitch switchReverse;
		//public final SparkMaxLimitSwitch switchForward;

		public WristJoint() {
			switchReverse = motor.getReverseLimitSwitch(Type.kNormallyOpen);
			//switchForward = motor.getForwardLimitSwitch(Type.kNormallyOpen);

			motor.setIdleMode(IdleMode.kBrake);
			motor.setInverted(true);

			encoder.setPositionConversionFactor(360 * (1 / Arm.GEAR_RATIO_WRIST));

		}

		public ProfiledPIDController controller = new ProfiledPIDController(0.7, 0.05, 0,
				new TrapezoidProfile.Constraints(VELOCITY, ACCELERATION));
		private final ArmFeedforward feedForward = new ArmFeedforward(0.01, 0, 0, 0);
		private MechanismLigament2d simulator;
	}

	public double degreesToEncoders(double angle) {
		return angle / 360 * ENCODERS_PER_REV;
	}

	public double encodersToRadians(double encoders) {
		return encoders / ENCODERS_PER_REV * (Math.PI * 2);
	}

	public static ArmChain armChainHorizontal = new ArmChain();
	public static ArmChain armChainVertical = new ArmChain();
	public static FabrikController armKinematics = new FabrikController();

	public ShoulderJoint shoulderJoint = new ShoulderJoint();
	public ElbowJoint elbowJoint = new ElbowJoint();
	public WristJoint wristJoint = new WristJoint();

	public final double ENCODERS_PER_REV = wristJoint.encoder.getCountsPerRevolution();

	private Mechanism2d armSimulation;
	public MechanismRoot2d effectorRoot;
	private MechanismLigament2d effectorPoint;

	private double shoulderAngle;
	private double elbowAngle;
	private double storedWristAngle;

	// left limit, right limit
	public float[] shoulderLimits = { -50, (float) 19.5 };
	public float[] elbowLimits = { -170, -30 };
	public float[] wristLimits = { -135, 135 };

	// private final double clawStartAngle;

	public ArmSubsystem() {

		SmartDashboard.putNumber("Neo counts per revolution", ENCODERS_PER_REV);

		// Initialize Inverse Kinematics with constant values
		armChainHorizontal.setArmLength(62f)
				.setSegmentLengthRatio(0, 31 / 62f)
				.setSegmentLengthRatio(1, 31 / 62f)
				.setSegmentLengths()
				.setAngleConstraint(0, -shoulderLimits[0], shoulderLimits[1])
				.setAngleConstraint(1, -elbowLimits[0], elbowLimits[1])
				.setSegmentInitialDirection(0, (float) Math.toRadians(90))
				.setSegmentInitialDirection(1, (float) Math.toRadians(0))
				.initializeChain()
				.addGloballyConstrainedGripper((float) Math.toRadians(0), 8f)
				.setSolveDistanceThreshold(1f)
				.setMaxIterationAttempts(5000);

		armChainVertical.setArmLength(62f)
				.setSegmentLengthRatio(0, 31 / 62f)
				.setSegmentLengthRatio(1, 31 / 62f)
				.setSegmentLengths()
				.setAngleConstraint(0, -shoulderLimits[0], shoulderLimits[1])
				.setAngleConstraint(1, -elbowLimits[0], elbowLimits[1])
				.setSegmentInitialDirection(0, (float) Math.toRadians(90))
				.setSegmentInitialDirection(1, (float) Math.toRadians(0))
				.initializeChain()
				.addGloballyConstrainedGripper((float) Math.toRadians(-90), 8f)
				.setSolveDistanceThreshold(1f)
				.setMaxIterationAttempts(5000);

		// armKinematics = new FabrikController(armChainHorizontal, armChainVertical);

		// clawJoint.clawEncoder.setPositionConversionFactor(Arm.GEAR_RATIO_CLAW);

		armSimulation = new Mechanism2d(300 / Arm.SIMULATION_SCALE, 300 / Arm.SIMULATION_SCALE);
		MechanismRoot2d armRoot = armSimulation.getRoot("Arm", (Arm.SIMULATION_OFFSET + 150) / Arm.SIMULATION_SCALE,
				(Arm.SIMULATION_OFFSET) / Arm.SIMULATION_SCALE);
		shoulderJoint.simulator = armRoot.append(new MechanismLigament2d("shoulder",
				(double) armChainHorizontal.getSegmentLength(0) / Arm.SIMULATION_SCALE, 0, 5.0,
				new Color8Bit(255, 0, 0)));
		elbowJoint.simulator = shoulderJoint.simulator.append(new MechanismLigament2d("elbow",
				(double) (armChainHorizontal.getSegmentLength(1)) / Arm.SIMULATION_SCALE, 0.0, 5.0,
				new Color8Bit(0, 255, 0)));
		wristJoint.simulator = elbowJoint.simulator.append(new MechanismLigament2d("claw",
				(double) 12 / Arm.SIMULATION_SCALE, 0, 5.0, new Color8Bit(0, 0, 255)));
		effectorRoot = armSimulation.getRoot("End Effector",
				(Arm.SIMULATION_OFFSET + 150) / Arm.SIMULATION_SCALE + cartesianStorage.getX(),
				(Arm.SIMULATION_OFFSET) / Arm.SIMULATION_SCALE + cartesianStorage.getZ());

		effectorPoint = effectorRoot.append(
				new MechanismLigament2d("effector", 1.0 / Arm.SIMULATION_SCALE, 0.0, 5.0, new Color8Bit(255, 0, 0)));
		shoulderAngle = armChainHorizontal.getIKAnglesDegrees().get(0);
		elbowAngle = shoulderAngle - armChainHorizontal.getIKAnglesDegrees().get(1);
		// reach(armKinematics.getIKAnglesRadians());

		// storedClawAngle = storedElbowAngle+armKinematics.getIKAnglesDegrees().get(2);
		SmartDashboard.putNumber("Counts per rev", shoulderJoint.leftEncoder.getCountsPerRevolution());
	}

	public static NetworkTable getArmNetwork() {
		if (armNetwork == null) {
			return NetworkTableInstance.getDefault().getTable("arm");
		}
		return armNetwork;
	}

	public void reach(Translation3d translation) {
		armKinematics.solveNewTarget(translation);
		// Units of error are inches

		if (Robot.operatorController.getRightTriggerAxis() < 0.9) {
			reach(armKinematics.getAnglesRadians(0));
		} else {
			reach(armKinematics.getAnglesRadians(1));
		}
	}

	public void reachEmbedded(Translation3d translation) {

		if (Robot.operatorController.getRightTriggerAxis() < 0.9) {
			armChainHorizontal.updateEmbedded((float) translation.getX(), (float) translation.getZ());
			armChainHorizontal.solveForEmbedded();

			List<Double> desiredAngles = armChainHorizontal.getIKAnglesRadians();

			reach(desiredAngles);
		} else {
			armChainVertical.updateEmbedded((float) translation.getX(), (float) translation.getZ());
			armChainVertical.solveForEmbedded();
			reach(armChainVertical.getIKAnglesRadians());
		}

	}

	public void reach(List<Double> desiredRadianAngles) {

		shoulderAngle = desiredRadianAngles.get(0);
		elbowAngle = desiredRadianAngles.get(1);

		System.out.println("Elbow Desired Angle: " + Math.toDegrees(desiredRadianAngles.get(1)));

		shoulderAngleEntry.setDouble(Math.toDegrees(desiredRadianAngles.get(0)));
		elbowAngleEntry.setDouble(Math.toDegrees(desiredRadianAngles.get(1)));
		wristAngleEntry.setDouble(Math.toDegrees(desiredRadianAngles.get(2)));

		// round to nearest degree
		double shoulderEncoders = Math.round(Math.toDegrees(desiredRadianAngles.get(0)));
		double elbowEncoders = Math.round(Math.toDegrees(desiredRadianAngles.get(1)));
		double wristEncoders =  Math.round(Math.toDegrees(desiredRadianAngles.get(2)));
		// * shoulderJoint.shoulderEncoder1.getCountsPerRevolution()) *
		// Arm.GEAR_RATIO_CLAW;

		shoulderEncoderEntrySim.setDouble(shoulderEncoders);
		elbowEncoderEntrySim.setDouble(elbowEncoders);
		// clawEncoderTab.setDouble(clawEncoders);
		// Gear ratio is 40 / 12 * 160 * ShoulderJoint.shoulderEncoder.getEncodersPer

		shoulderJoint.controller.setGoal(Math.toRadians(shoulderEncoders));
		elbowJoint.controller.setGoal(Math.toRadians(elbowEncoders));
		wristJoint.controller.setGoal(Math.toRadians(wristEncoders));

		double shoulderFeedForward = shoulderJoint.feedForward.calculate(
				shoulderJoint.controller.getGoal().position, shoulderJoint.controller.getGoal().velocity);
		double elbowFeedForward = elbowJoint.feedForward.calculate(elbowJoint.controller.getGoal().position,
				elbowJoint.controller.getGoal().velocity);
		double wristFeedForward = wristJoint.feedForward.calculate(wristJoint.controller.getGoal().position,
				wristJoint.controller.getGoal().velocity);

		double shoulderFeedback = shoulderJoint.controller.calculate(
				Math.toRadians(shoulderJoint.leftEncoder.getPosition()),
				shoulderJoint.controller.getGoal());
		double elbowFeedback = elbowJoint.controller.calculate(
				Math.toRadians(elbowJoint.encoder.getPosition()),
				elbowJoint.controller.getGoal());
		double wristFeedback = wristJoint.controller.calculate(
				Math.toRadians(wristJoint.encoder.getPosition()),
				wristJoint.controller.getGoal());

		// shoulderGoalPos.setDouble(shoulderJoint.shoulderControl.getGoal().position);
		// shoulderGoalVel.setDouble(shoulderJoint.shoulderControl.getGoal().velocity);
		// elbowGoalPos.setDouble(elbowJoint.elbowControl.getGoal().position);
		// elbowGoalVel.setDouble(elbowJoint.elbowControl.getGoal().velocity);
		// double clawFeedback = clawJoint.clawControl.calculate(
		// Units.rotationsToRadians(clawJoint.clawEncoder.getPosition()),
		// clawJoint.clawControl.getGoal());

		// if (shoulderJoint.shoulderEncoder1.getPosition() < shoulderLimits[0] ||
		// shoulderJoint.shoulderEncoder1.getPosition() > shoulderLimits[1]) {
		// shoulderJoint.setMotors(-(shoulderFeedForward + shoulderFeedback) * 0.05);
		// System.out.println("Shoulder hit soft limit!!!");
		// } else {
		shoulderJoint.setMotors(
				shoulderFeedForward +
						shoulderFeedback);
		// }

		// if (elbowJoint.elbowEncoder.getPosition() < elbowLimits[0] ||
		// elbowJoint.elbowEncoder.getPosition() > elbowLimits[1]) {
		// elbowJoint.elbowMotor.set(-(elbowFeedForward + elbowFeedback) * 0.05);
		// System.out.println("Elbow hit soft limit!!!");
		// } else {

		// if (elbowJoint.encoder.getPosition() <= elbowLimits[0]) {
		// 	elbowJoint.motor.set(
		// 		(-elbowFeedForward -
		// 				elbowFeedback) * 0.6);
		// } else {
			elbowJoint.motor.set(
					elbowFeedForward +
							elbowFeedback);
		// }
		// }

		wristJoint.motor.set((wristFeedForward + wristFeedback));

		// if (clawJoint.clawEncoder.getPosition() < degreesToEncoders(wristLimits[0],
		// Arm.GEAR_RATIO_CLAW) ||
		// clawJoint.clawEncoder.getPosition() > degreesToEncoders(wristLimits[0],
		// Arm.GEAR_RATIO_CLAW)) {
		// clawJoint.clawMotor.set(-(clawFeedForward + clawFeedback) * 0.02);
		// } else {
		// clawJoint.clawMotor.set((clawFeedForward + clawFeedback) * 0.02);
		// }
		shoulderSpeed.setDouble(
				shoulderFeedForward +
						shoulderFeedback);
		elbowSpeed.setDouble(
				elbowFeedForward +
						elbowFeedback);
	}

	public void resetEncoders() {
		shoulderJoint.leftEncoder.setPosition(0.0);
		elbowJoint.encoder.setPosition(0.0);
		wristJoint.encoder.setPosition(0.0);
	}

	public void setEncoders(double shoulder, double elbow, double wrist) {
		shoulderJoint.leftEncoder.setPosition(shoulder);
		shoulderJoint.rightEncoder.setPosition(shoulder);
		shoulderJoint.controller.reset(Math.toRadians(shoulder));

		elbowJoint.encoder.setPosition(elbow);
		elbowJoint.controller.reset(Math.toRadians(elbow));

		wristJoint.encoder.setPosition(wrist);
		wristJoint.controller.reset(Math.toRadians(wrist));
	}

	// public void setClawAngleConstraint(float angleRadians) {
	// armChainHorizontal.setGripperGlobalConstraint(angleRadians);
	// }

	// public void toggleClaw() {
	// if (armKinematics.getGripperGlobalConstraint() == 0) {
	// setClawAngleConstraint((float) Math.PI/2);
	// } else {
	// setClawAngleConstraint(0);
	// }
	// }

	public double[] getEncoderPosition() {
		return new double[] {
				(shoulderJoint.leftEncoder.getPosition() + shoulderJoint.rightEncoder.getPosition()) / 2,
				elbowJoint.encoder.getPosition(),
				wristJoint.encoder.getPosition() };
	}

	public double[] getJointRevolutions() {
		return new double[] { shoulderJoint.leftEncoder.getCountsPerRevolution(),
				elbowJoint.encoder.getCountsPerRevolution(), wristJoint.encoder.getCountsPerRevolution() };
	}

	// public void resetEncodersNew() {
	// double shoulderStartAngle = (Math.toRadians(Math.round(-20)) / (Math.PI * 2))
	// * shoulderJoint.shoulderEncoder1.getCountsPerRevolution();

	// double elbowStartAngle = (Math.toRadians(Math.round(170.0)) / (Math.PI * 2))
	// * elbowJoint.elbowEncoder.getCountsPerRevolution();

	// setEncoders(shoulderStartAngle, elbowStartAngle, 0);
	// }

	public float getPositionError() {
		return armChainHorizontal.getIKPositionError();
	}

	public void move(double shoulderSpeed, double elbowSpeed, double wristSpeed) {
		/* Shoulder */

		boolean forwardShoulderLimit = Math.signum(shoulderSpeed) > 0
				&& false; // TODO: Limit based on encoders
		boolean reverseShoulderLimit = Math.signum(shoulderSpeed) < 0
				&& (shoulderJoint.switchReverse.isPressed());

		if (!forwardShoulderLimit && !reverseShoulderLimit) {
			shoulderJoint.leftMotor.set(shoulderSpeed);
			shoulderJoint.rightMotor.set(shoulderSpeed);
		}

		/* Elbow */

		boolean forwardElbowLimit = Math.signum(elbowSpeed) > 0
				&& (elbowJoint.switchForward.isPressed());
		boolean reverseElbowLimit = Math.signum(elbowSpeed) < 0
				&& (elbowJoint.switchReverse.isPressed());

		
		if (!reverseElbowLimit && !forwardElbowLimit) {
				elbowJoint.motor.set(elbowSpeed);
		}
		
		/* Wrist */

		//boolean forwardWristLimit = Math.signum(wristSpeed) > 0
				//&& (wristJoint.switchForward.isPressed());
		boolean reverseWristLimit = Math.signum(wristSpeed) < 0
				&& (wristJoint.switchReverse.isPressed());


		if (!reverseWristLimit) {
		 	wristJoint.motor.set(wristSpeed);
		}

		/* Wrist */

		wristJoint.motor.set(wristSpeed);
	}

	@Override
	public void periodic() {
		JOINT_ANGLES_PUB.set(JOINT_ANGLES_SUB.get(new double[] { 0.0, 0.0 }));
		JOINT_RPM_PUB.set(JOINT_RPM_SUB.get(new double[] { 0.0, 0.0 }));
		double shoulderSimAngle;
		double elbowSimAngle;
		double wristSimAngle;

		if (Robot.operatorController.getRightTriggerAxis() < 0.9) {
			shoulderSimAngle = 90 - armChainHorizontal.getIKAnglesDegrees().get(0);
			elbowSimAngle = -armChainHorizontal.getIKAnglesDegrees().get(1);
			wristSimAngle = -armChainHorizontal.getIKAnglesDegrees().get(2);
		} else {
			shoulderSimAngle = 90 - armChainVertical.getIKAnglesDegrees().get(0);
			elbowSimAngle = -armChainVertical.getIKAnglesDegrees().get(1);
			wristSimAngle = -armChainVertical.getIKAnglesDegrees().get(2);
		}

		if (shoulderSimAngle != shoulderAngle || elbowSimAngle != elbowAngle
				|| wristSimAngle != storedWristAngle) {
			shoulderAngle = shoulderSimAngle;
			elbowAngle = elbowSimAngle;
			storedWristAngle = wristSimAngle;
			// storedClawAngle = clawSimAngle;
			// double absoluteElbow = storedShoulderAngle + storedElbowAngle;
			// double absoluteClaw = storedClawAngle + storedShoulderAngle +
			// storedElbowAngle;
			// double targetAngle = Math.toDegrees(Math.atan2(cartesianStorage.getZ(),
			// cartesianStorage.getX()));

			// shoulderAngleAbsolute.setDouble(storedShoulderAngle);
			// jointAngleAbsolute.setDouble(absoluteElbow);
			// clawAngleAbsolute.setDouble(absoluteClaw);
		}

		// if (shoulderJoint.shoulderSwitch2.isPressed() ||
		// shoulderJoint.shoulderSwitch1.isPressed()) {
		// // CommandScheduler.getInstance().cancelAll();
		// shoulderJoint.setMotors(0);
		// }
		// if (elbowJoint.elbowSwitch.isPressed()) {
		// // CommandScheduler.getInstance().cancelAll();
		// elbowJoint.elbowMotor.set(0);
		// }
		// if (clawJoint.clawSwitch.isPressed()) {
		// CommandScheduler.getInstance().cancel(new DefaultIKControlCommand(true));
		// clawJoint.clawMotor.set(0);
		// }

		shoulderJoint.simulator.setAngle(shoulderAngle);
		elbowJoint.simulator.setAngle(elbowAngle);
		wristJoint.simulator.setAngle(storedWristAngle);
		// effectorAngle.setDouble(Math.toDegrees(Math.atan2(cartesianStorage.getZ(),
		// cartesianStorage.getX())));
		effectorX.setDouble(cartesianStorage.getX());
		effectorY.setDouble(cartesianStorage.getZ());
		shoulderEncoderEntry.setDouble(shoulderJoint.leftEncoder.getPosition());
		elbowEncoderEntry.setDouble(elbowJoint.encoder.getPosition());
		wristEncoderEntry.setDouble(wristJoint.encoder.getPosition());

		shoulderSwitchReverseEntry.setBoolean(shoulderJoint.switchReverse.isPressed());
		elbowSwitchForwardTab.setBoolean(elbowJoint.switchForward.isPressed());
		wristSwitchReverseTab.setBoolean(wristJoint.switchReverse.isPressed());

		Logger.getInstance().recordOutput("MyMechanism", this.armSimulation);
	}
}