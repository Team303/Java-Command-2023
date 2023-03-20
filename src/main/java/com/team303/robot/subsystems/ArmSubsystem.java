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
import com.team303.robot.RobotMap.Arm;

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

	public static final GenericEntry shoulderAngle = ARM_TAB.add("Shoulder Angle", 0).withPosition(0, 0).getEntry();
	public static final GenericEntry elbowAngle = ARM_TAB.add("Elbow Angle", 0).withPosition(1, 0).getEntry();
	public static final GenericEntry wristAngle = ARM_TAB.add("Wrist Angle", 0).withPosition(2, 0).getEntry();

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
	public static final GenericEntry shoulder1SwitchReverseTab = ARM_TAB.add("shoulder1 switch", false)
			.withPosition(2, 1).getEntry();
	public static final GenericEntry shoulder2SwitchReverseTab = ARM_TAB.add("shoulder2 switch", false)
			.withPosition(3, 1).getEntry();
	public static final GenericEntry elbowSwitchReverseTab = ARM_TAB.add("elbow switch", false).withPosition(2, 2)
			.getEntry();
	public static final GenericEntry shoulder1SwitchForwardTab = ARM_TAB.add("shoulder1 switch forward ", false)
			.withPosition(3, 2).getEntry();
	public static final GenericEntry shoulder2SwitchForwardTab = ARM_TAB.add("shoulder2 switch forward", false)
			.withPosition(2, 3).getEntry();
	public static final GenericEntry elbowSwitchForwardTab = ARM_TAB.add("elbow switch forward", false)
			.withPosition(3, 3).getEntry();
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
	public static final double CIRCUMFERENCE = 2 * Math.PI * ARMLEN_METERS;

	// rotations/sec
	public static final double VELOCITY = 0.1;

	// rotations/sec^2
	public static final double ACCELERATION = 0.05;

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

		public final SparkMaxLimitSwitch leftSwitchReverse;
		public final SparkMaxLimitSwitch rightSwitchReverse;

		public final SparkMaxLimitSwitch leftSwitchForward;
		public final SparkMaxLimitSwitch rightSwitchForward;

		public ShoulderJoint() {
			leftMotor.setIdleMode(IdleMode.kBrake);
			rightMotor.setIdleMode(IdleMode.kBrake);

			leftSwitchReverse = leftMotor.getReverseLimitSwitch(Type.kNormallyOpen);
			rightSwitchReverse = rightMotor.getReverseLimitSwitch(Type.kNormallyOpen);

			leftSwitchForward = leftMotor.getForwardLimitSwitch(Type.kNormallyOpen);
			rightSwitchForward = rightMotor.getForwardLimitSwitch(Type.kNormallyOpen);

			leftMotor.setInverted(true);
			rightMotor.setInverted(false);

			leftEncoder.setPositionConversionFactor(360 * (1 / Arm.GEAR_RATIO_SHOULDER));
			rightEncoder.setPositionConversionFactor(360 * (1 / Arm.GEAR_RATIO_SHOULDER));
		}

		public final void setMotors(double speed) {
			leftMotor.set(speed);
			rightMotor.set(speed);
		}

		public final ProfiledPIDController controller = new ProfiledPIDController(0.01, 0, 0,
				new TrapezoidProfile.Constraints(CIRCUMFERENCE * VELOCITY, CIRCUMFERENCE * ACCELERATION));
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

		public ProfiledPIDController controller = new ProfiledPIDController(0.01, 0, 0,
				new TrapezoidProfile.Constraints(CIRCUMFERENCE * VELOCITY, CIRCUMFERENCE * ACCELERATION));
		private final ArmFeedforward feedForward = new ArmFeedforward(0.01, 0, 0, 0);
		private MechanismLigament2d simulator;
	}

	public class WristJoint {
		private final CANSparkMax motor = new CANSparkMax(Arm.CLAW_JOINT_ID, MotorType.kBrushless);
		private final RelativeEncoder encoder = motor.getEncoder();

		public final SparkMaxLimitSwitch switchReverse;
		public final SparkMaxLimitSwitch switchForward;

		public WristJoint() {
			switchReverse = motor.getReverseLimitSwitch(Type.kNormallyOpen);
			switchForward = motor.getForwardLimitSwitch(Type.kNormallyOpen);

			motor.setIdleMode(IdleMode.kBrake);
			motor.setInverted(true);
		}

		public ProfiledPIDController controller = new ProfiledPIDController(0.01, 0, 0,
				new TrapezoidProfile.Constraints(CIRCUMFERENCE * VELOCITY, CIRCUMFERENCE * ACCELERATION));
		private final ArmFeedforward feedForward = new ArmFeedforward(0.01, 0, 0, 0);
		private MechanismLigament2d simulator;
	}

	public double degreesToEncoders(double angle) {
		return angle / 360 * ENCODERS_PER_REV;
	}

	public double encodersToRadians(double encoders) {
		return encoders / ENCODERS_PER_REV * (Math.PI * 2);
	}

	public static FabrikController armKinematics = new FabrikController();

	public ShoulderJoint shoulderJoint = new ShoulderJoint();
	public ElbowJoint elbowJoint = new ElbowJoint();
	public WristJoint wristJoint = new WristJoint();

	public final double ENCODERS_PER_REV = wristJoint.encoder.getCountsPerRevolution();

	private Mechanism2d armSimulation;
	public MechanismRoot2d effectorRoot;
	private MechanismLigament2d effectorPoint;

	private double storedShoulderAngle;
	private double storedElbowAngle;
	private double storedClawAngle;

	// left limit, right limit
	public float[] shoulderLimits = { -20, 20 };
	public float[] elbowLimits = { -170, -45 };
	public float[] wristLimits = { -135, 135 };

	// private final double clawStartAngle;

	public ArmSubsystem() {

		SmartDashboard.putNumber("Neo counts per revolution", ENCODERS_PER_REV);

		// Initialize Inverse Kinematics with constant values
		armKinematics.setArmLength(74f);
		armKinematics.setSegmentLengthRatio(0, 31 / 74f);
		armKinematics.setSegmentLengthRatio(1, (31 + 12) / 74f);
		// armKinematics.setSegmentLengthRatio(2, 12 / 74f);
		armKinematics.setSegmentLengths();
		armKinematics.setAngleConstraint(0, -shoulderLimits[0], shoulderLimits[1]);
		armKinematics.setAngleConstraint(1, -elbowLimits[0], elbowLimits[1]);
		// armKinematics.setAngleConstraint(2, -wristLimits[0], wristLimits[0]);
		armKinematics.setSegmentInitialDirection(0, (float) Math.toRadians(90));
		armKinematics.setSegmentInitialDirection(1, (float) Math.toRadians(0));
		// armKinematics.setSegmentInitialDirection(2, (float) Math.toRadians(-45));
		armKinematics.initializeArm();
		armKinematics.setSolveDistanceThreshold(1f);
		armKinematics.setMaxIterationAttempts(5000);
		// clawJoint.clawEncoder.setPositionConversionFactor(Arm.GEAR_RATIO_CLAW);

		armSimulation = new Mechanism2d(300 / Arm.SIMULATION_SCALE, 300 / Arm.SIMULATION_SCALE);
		MechanismRoot2d armRoot = armSimulation.getRoot("Arm", (Arm.SIMULATION_OFFSET + 150) / Arm.SIMULATION_SCALE,
				(Arm.SIMULATION_OFFSET) / Arm.SIMULATION_SCALE);
		shoulderJoint.simulator = armRoot.append(new MechanismLigament2d("shoulder",
				(double) armKinematics.getSegmentLength(0) / Arm.SIMULATION_SCALE, 0, 5.0, new Color8Bit(255, 0, 0)));
		elbowJoint.simulator = shoulderJoint.simulator.append(new MechanismLigament2d("elbow",
				(double) armKinematics.getSegmentLength(1) / Arm.SIMULATION_SCALE, 0.0, 5.0, new Color8Bit(0, 255, 0)));
		// clawJoint.clawSimulator = elbowJoint.elbowSimulator.append(new
		// MechanismLigament2d("claw",
		// (double) armKinematics.getSegmentLength(2)/Arm.SIMULATION_SCALE, 0, 5.0, new
		// Color8Bit(0, 0, 255)));
		effectorRoot = armSimulation.getRoot("End Effector",
				(Arm.SIMULATION_OFFSET + 150) / Arm.SIMULATION_SCALE + cartesianStorage.getX(),
				(Arm.SIMULATION_OFFSET) / Arm.SIMULATION_SCALE + cartesianStorage.getZ());

		effectorPoint = effectorRoot.append(
				new MechanismLigament2d("effector", 1.0 / Arm.SIMULATION_SCALE, 0.0, 5.0, new Color8Bit(255, 0, 0)));
		storedShoulderAngle = armKinematics.getIKAnglesDegrees().get(0);
		storedElbowAngle = storedShoulderAngle - armKinematics.getIKAnglesDegrees().get(1);
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
		armKinematics.solveTargetIK(translation);
		// Units of error are inches
		reach(armKinematics.getIKAnglesRadians());
	}

	public void reachEmbedded(Translation3d translation) {
		armKinematics.solveForEmbedded();

		reach(armKinematics.getIKAnglesRadians());
	}

	public void reach(List<Double> desiredRadianAngles) {

		storedShoulderAngle = desiredRadianAngles.get(0);
		storedElbowAngle = desiredRadianAngles.get(1);

		shoulderAngle.setDouble(Math.toDegrees(desiredRadianAngles.get(0)));
		elbowAngle.setDouble(Math.toDegrees(desiredRadianAngles.get(1)));
		// clawAngle.setDouble(Math.toDegrees(desiredRadianAngles.get(2)));

		double shoulderEncoders = Math.round((desiredRadianAngles.get(0)) / (Math.PI * 2) * 360);
		double elbowEncoders = Math.round((desiredRadianAngles.get(1)) / (Math.PI * 2) * 360);
		// double clawEncoders = Math.round((desiredRadianAngles.get(2)) / (Math.PI * 2)
		// * shoulderJoint.shoulderEncoder1.getCountsPerRevolution()) *
		// Arm.GEAR_RATIO_CLAW;

		shoulderEncoderEntrySim.setDouble(shoulderEncoders);
		elbowEncoderEntrySim.setDouble(elbowEncoders);
		// clawEncoderTab.setDouble(clawEncoders);
		// Gear ratio is 40 / 12 * 160 * ShoulderJoint.shoulderEncoder.getEncodersPer

		shoulderJoint.controller.setGoal(Math.toRadians(shoulderEncoders) * CIRCUMFERENCE);
		elbowJoint.controller.setGoal(Math.toRadians(elbowEncoders) * CIRCUMFERENCE);
		// clawJoint.clawControl.setGoal(clawEncoders);
		double shoulderFeedForward = shoulderJoint.feedForward.calculate(
				shoulderJoint.controller.getGoal().position, shoulderJoint.controller.getGoal().velocity);
		double elbowFeedForward = elbowJoint.feedForward.calculate(elbowJoint.controller.getGoal().position,
				elbowJoint.controller.getGoal().velocity);
		// double clawFeedForward =
		// clawJoint.m_clawFeedForward.calculate(clawJoint.clawControl.getGoal().position,
		// clawJoint.clawControl.getGoal().velocity);

		double shoulderFeedback = shoulderJoint.controller.calculate(
				Math.toRadians(shoulderJoint.leftEncoder.getPosition()) * CIRCUMFERENCE,
				shoulderJoint.controller.getGoal());
		double elbowFeedback = elbowJoint.controller.calculate(
				Math.toRadians(elbowJoint.encoder.getPosition()) * CIRCUMFERENCE,
				elbowJoint.controller.getGoal());

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
		shoulderJoint.setMotors((shoulderFeedForward + shoulderFeedback));
		// }

		// if (elbowJoint.elbowEncoder.getPosition() < elbowLimits[0] ||
		// elbowJoint.elbowEncoder.getPosition() > elbowLimits[1]) {
		// elbowJoint.elbowMotor.set(-(elbowFeedForward + elbowFeedback) * 0.05);
		// System.out.println("Elbow hit soft limit!!!");
		// } else {
		elbowJoint.motor.set((elbowFeedForward + elbowFeedback));
		// }

		// if (clawJoint.clawEncoder.getPosition() < degreesToEncoders(wristLimits[0],
		// Arm.GEAR_RATIO_CLAW) ||
		// clawJoint.clawEncoder.getPosition() > degreesToEncoders(wristLimits[0],
		// Arm.GEAR_RATIO_CLAW)) {
		// clawJoint.clawMotor.set(-(clawFeedForward + clawFeedback) * 0.02);
		// } else {
		// clawJoint.clawMotor.set((clawFeedForward + clawFeedback) * 0.02);
		// }
		shoulderSpeed.setDouble(shoulderFeedForward + shoulderFeedback);
		elbowSpeed.setDouble(elbowFeedForward + elbowFeedback);
	}

	public void resetEncoders() {
		shoulderJoint.leftEncoder.setPosition(0.0);
		elbowJoint.encoder.setPosition(0.0);
		wristJoint.encoder.setPosition(0.0);
	}

	public void setEncoders(double shoulder, double elbow, double wrist) {
		shoulderJoint.leftEncoder.setPosition(shoulder);
		shoulderJoint.rightEncoder.setPosition(shoulder);
		elbowJoint.encoder.setPosition(elbow);
		wristJoint.encoder.setPosition(wrist);
	}

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
		return armKinematics.getIKPositionError();
	}

	public void move(double shoulderSpeed, double elbowSpeed, double wristSpeed) {
		/* Shoulder */

		boolean forwardShoulderLimit = Math.signum(shoulderSpeed) > 0
				&& (shoulderJoint.leftSwitchForward.isPressed()
						|| shoulderJoint.rightSwitchForward.isPressed());
		boolean reverseShoulderLimit = Math.signum(shoulderSpeed) < 0
				&& (shoulderJoint.leftSwitchReverse.isPressed()
						|| shoulderJoint.rightSwitchReverse.isPressed());

		if (!forwardShoulderLimit && !reverseShoulderLimit) {
			shoulderJoint.leftMotor.set(shoulderSpeed);
			shoulderJoint.rightMotor.set(shoulderSpeed);
		}

		/* Elbow */

		boolean forwardElbowLimit = Math.signum(elbowSpeed) > 0
				&& (elbowJoint.switchForward.isPressed());
		boolean reverseElbowLimit = Math.signum(elbowSpeed) < 0
				&& (elbowJoint.switchReverse.isPressed());

		if (!forwardElbowLimit && !reverseElbowLimit) {
			elbowJoint.motor.set(elbowSpeed);
		}

		/* Wrist */

		wristJoint.motor.set(wristSpeed);
	}

	@Override
	public void periodic() {
		JOINT_ANGLES_PUB.set(JOINT_ANGLES_SUB.get(new double[] { 0.0, 0.0 }));
		JOINT_RPM_PUB.set(JOINT_RPM_SUB.get(new double[] { 0.0, 0.0 }));
		double shoulderSimAngle = 90 - armKinematics.getIKAnglesDegrees().get(0);
		double elbowSimAngle = -armKinematics.getIKAnglesDegrees().get(1);
		// double clawSimAngle = -armKinematics.getIKAnglesDegrees().get(2);

		if (shoulderSimAngle != storedShoulderAngle || elbowSimAngle != storedElbowAngle /*
																							 * || clawSimAngle !=
																							 * storedClawAngle
																							 */) {
			storedShoulderAngle = shoulderSimAngle;
			storedElbowAngle = elbowSimAngle;
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

		shoulderJoint.simulator.setAngle(storedShoulderAngle);
		elbowJoint.simulator.setAngle(storedElbowAngle);
		// clawJoint.clawSimulator.setAngle(storedClawAngle);
		// effectorAngle.setDouble(Math.toDegrees(Math.atan2(cartesianStorage.getZ(),
		// cartesianStorage.getX())));
		effectorX.setDouble(cartesianStorage.getX());
		effectorY.setDouble(cartesianStorage.getZ());
		shoulderEncoderEntry.setDouble(shoulderJoint.leftEncoder.getPosition());
		elbowEncoderEntry.setDouble(elbowJoint.encoder.getPosition());

		shoulder1SwitchReverseTab.setBoolean(shoulderJoint.leftSwitchReverse.isPressed());
		shoulder2SwitchReverseTab.setBoolean(shoulderJoint.rightSwitchReverse.isPressed());
		shoulder1SwitchForwardTab.setBoolean(shoulderJoint.leftSwitchForward.isPressed());
		shoulder2SwitchForwardTab.setBoolean(shoulderJoint.rightSwitchForward.isPressed());
		elbowSwitchForwardTab.setBoolean(elbowJoint.switchForward.isPressed());
		elbowSwitchReverseTab.setBoolean(elbowJoint.switchReverse.isPressed());

		Logger.getInstance().recordOutput("MyMechanism", this.armSimulation);
	}
}
