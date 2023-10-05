// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team303.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.team303.lib.kinematics.ArmChain;
import com.team303.lib.kinematics.FabrikController;
import com.team303.robot.Robot;
import com.team303.robot.RobotMap.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.List;

import static com.team303.robot.commands.arm.DefaultIKControlCommand.cartesianStorage;
import static com.team303.robot.Robot.manipulator;

public class ArmSubsystem extends SubsystemBase {
	public static boolean isVerticalChain2 = false; 

	public static final ShuffleboardTab ARM_TAB = Shuffleboard.getTab("Arm");

	/* Row 1 */

	public static final GenericEntry shoulderAngleEntry = ARM_TAB.add("Desired Shoulder Angle", 0).withSize(1, 1)
			.withPosition(0, 0).getEntry();
	public static final GenericEntry elbowAngleEntry = ARM_TAB.add("Desired Elbow Angle", 0).withSize(1, 1)
			.withPosition(1, 0).getEntry();
	public static final GenericEntry wristAngleEntry = ARM_TAB.add("Desired Wrist Angle", 0).withSize(1, 1)
			.withPosition(2, 0).getEntry();

	public static final GenericEntry effectorXEntry = ARM_TAB.add("effectorX", 0).withSize(1, 1).withPosition(3, 0)
			.getEntry();
	public static final GenericEntry effectorYEntry = ARM_TAB.add("effectorY", 0).withSize(1, 1).withPosition(4, 0)
			.getEntry();

	/* Row 2 */

	public static final GenericEntry shoulderEncoderEntry = ARM_TAB.add("Shoulder Encoder", 0).withSize(1, 1)
			.withPosition(0, 1)
			.getEntry();
	public static final GenericEntry elbowEncoderEntry = ARM_TAB.add("Elbow Encoder", 0).withSize(1, 1)
			.withPosition(1, 1).getEntry();
	public static final GenericEntry wristEncoderEntry = ARM_TAB.add("Wrist Encoder", 0).withSize(1, 1)
			.withPosition(2, 1).getEntry();

	/* Row 3 */

	public static final GenericEntry shoulderAbsoluteAngleEntry = ARM_TAB.add("Shoulder Absolute Angle", 0)
			.withSize(1, 1).withPosition(0, 2).getEntry();
	public static final GenericEntry elbowAbsoluteAngleEntry = ARM_TAB.add("Elbow Absolute Angle", 0).withSize(1, 1)
			.withPosition(1, 2).getEntry();
	public static final GenericEntry wristAbsoluteAngleEntry = ARM_TAB.add("Wrist Absolute Angle", 0).withSize(1, 1)
			.withPosition(2, 2).getEntry();

	/* Row 4 */

	public static final GenericEntry shoulderSwitchEntry = ARM_TAB.add("Shoulder Switch", false).withSize(1, 1)
			.withPosition(0, 3).getEntry();
	public static final GenericEntry elbowSwitchEntry = ARM_TAB.add("Elbow Switch", false).withSize(1, 1)
			.withPosition(1, 3).getEntry();
	public static final GenericEntry wristSwitchEntry = ARM_TAB.add("Wrist Switch", false).withSize(1, 1)
			.withPosition(2, 3).getEntry();

	/* Row 5 */
	public static final GenericEntry shoulderSpeedEntry = ARM_TAB.add("Shoulder Speed", 0).withSize(1, 1)
			.withPosition(0, 4).getEntry();
	public static final GenericEntry elbowSpeedEntry = ARM_TAB.add("Elbow Speed", 0).withSize(1, 1).withPosition(1, 4)
			.getEntry();
	public static final GenericEntry wristSpeedEntry = ARM_TAB.add("Wrist Speed", 0).withSize(1, 1).withPosition(2, 4)
			.getEntry();

	/* Row 1 Part 2 */

	public static final GenericEntry shoulderSoftLimitEntry = ARM_TAB.add("Shoulder Soft Limit", false).withSize(1, 1)
			.withPosition(3, 0).getEntry();
	public static final GenericEntry elbowSoftLimitEntry = ARM_TAB.add("Elbow Soft Limit", false).withSize(1, 1)
			.withPosition(4, 0).getEntry();
	public static final GenericEntry wristSoftLimitEntry = ARM_TAB.add("Wrist Soft Limit", false).withSize(1, 1)
			.withPosition(5, 0).getEntry();

	/* Row 2 Part 2 */

	public static final GenericEntry shoulderSoftForwardLimitEntry = ARM_TAB.add("Shoulder Soft Forward Limit", false)
			.withSize(1, 1).withPosition(3, 1).getEntry();
	public static final GenericEntry elbowSoftForwardLimitEntry = ARM_TAB.add("Elbow Soft Forward Limit", false)
			.withSize(1, 1).withPosition(4, 1).getEntry();
	public static final GenericEntry wristSoftForwardLimitEntry = ARM_TAB.add("Wrist Soft Forward Limit", false)
			.withSize(1, 1).withPosition(5, 1).getEntry();

	/* Row 3 Part 2 */

	public static final GenericEntry shoulderSoftReverseLimitEntry = ARM_TAB.add("Shoulder Soft Reverse Limit", false)
			.withSize(1, 1).withPosition(3, 2).getEntry();
	public static final GenericEntry elbowSoftReverseLimitEntry = ARM_TAB.add("Elbow Soft Reverse Limit", false)
			.withSize(1, 1).withPosition(4, 2).getEntry();
	public static final GenericEntry wristSoftReverseLimitEntry = ARM_TAB.add("Wrist Soft Reverse Limit", false)
			.withSize(1, 1).withPosition(5, 2).getEntry();

	/* Row 4 Part 2 */

	public static final GenericEntry shoulderEncoderErrorEntry = ARM_TAB.add("Shoulder Encoder Error", false)
			.withSize(1, 1).withPosition(3, 3).getEntry();
	public static final GenericEntry elbowEncoderErrorEntry = ARM_TAB.add("Elbow Encoder Error", false).withSize(1, 1)
			.withPosition(4, 3).getEntry();
	public static final GenericEntry wristEncoderErrorEntry = ARM_TAB.add("Wrist Encoder Error", false).withSize(1, 1)
			.withPosition(5, 3).getEntry();

	/* relative to floor */

	public static final GenericEntry shoulderAbsoluteAngleFloorEntry = ARM_TAB.add("Shoulder Absolute Floor Angle", 0)
			.withSize(1, 1).withPosition(3, 4).getEntry();
	public static final GenericEntry elbowAbsoluteAngleFloorEntry = ARM_TAB.add("Elbow Absolute Floor Angle", 0)
			.withSize(1, 1)
			.withPosition(4, 4).getEntry();
	public static final GenericEntry wristAbsoluteAngleFloorEntry = ARM_TAB.add("Wrist Absolute Floor Angle", 0)
			.withSize(1, 1)
			.withPosition(5, 4).getEntry();

	// radians/sec
	public static final double MAX_VELOCITY = (2 * Math.PI) * 0.35;
	public static final double MAX_VELOCITY_ELBOW = (2 * Math.PI) * 0.4;
	public static final double MAX_VELOCITY_WRIST = MAX_VELOCITY_ELBOW;

	// radians/sec^2
	public static final double MAX_ACCELERATION = 64;

	public interface ArmJoint {
		/**
		 * Sets the speed of the motors
		 * Range is [-1, 1]
		 */
		void setSpeed(double speed);

		/**
		 * Gets the absolute angle of the joint in radians
		 */
		double getJointAngle();

		/**
		 * Gets the motor encoder position in radians
		 */
		double getEncoderPosition();

		/**
		 * Gets the state of the limit switch
		 */
		boolean atHardLimit();

		/**
		 * Computes whether the joint is outside the soft angle constraints
		 */
		default boolean atSoftLimit() {
			return atSoftForwardLimit() || atSoftReverseLimit();
		}

		/**
		 * Returns whether or not the joint is outside its soft forward limit
		 */
		boolean atSoftForwardLimit();

		/**
		 * Returns whether or not the joint is outside its soft reverse limit
		 */
		boolean atSoftReverseLimit();
	}

	public static class ShoulderJoint implements ArmJoint {

		/* Motors */

		private final CANSparkMax leftMotor = new CANSparkMax(Arm.SHOULDER_JOINT_LEFT_ID, MotorType.kBrushless);
		private final CANSparkMax rightMotor = new CANSparkMax(Arm.SHOULDER_JOINT_RIGHT_ID, MotorType.kBrushless);

		/* Encoders */

		private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
		private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

		private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(8);

		/* Limits */

		private final SparkMaxLimitSwitch switchReverse = rightMotor.getReverseLimitSwitch(Type.kNormallyOpen);

		/* Controllers */

		private final ProfiledPIDController controller = new ProfiledPIDController(0.6, 0, 0.05,
				new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));

		private final ArmFeedforward feedForward = new ArmFeedforward(0.01, 0, 0, 0);
		private MechanismLigament2d simulator;
		private MechanismLigament2d real;

		public ShoulderJoint() {
			leftMotor.setIdleMode(IdleMode.kBrake);
			rightMotor.setIdleMode(IdleMode.kBrake);

			leftMotor.setInverted(true);
			rightMotor.setInverted(false);

			// leftMotor.setSmartCurrentLimit(30);
			// rightMotor.setSmartCurrentLimit(30);

			leftEncoder.setPositionConversionFactor(2 * Math.PI * (1 / Arm.GEAR_RATIO_SHOULDER));
			rightEncoder.setPositionConversionFactor(2 * Math.PI * (1 / Arm.GEAR_RATIO_SHOULDER));

			controller.setTolerance(Math.toRadians(2));
		}

		@Override
		public void setSpeed(double speed) {
			leftMotor.set(speed);
			rightMotor.set(speed);
		}

		@Override
		public double getJointAngle() {
			return MathUtil.angleModulus(Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition())
					+ Math.toRadians(SHOULDER_START_ANGLE));
		}

		@Override
		public double getEncoderPosition() {
			return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
		}

		@Override
		public boolean atHardLimit() {
			return switchReverse.isPressed();
		}

		@Override
		public boolean atSoftForwardLimit() {
			return this.getJointAngle() > Math.toRadians(shoulderLimits[0]);
		}

		@Override
		public boolean atSoftReverseLimit() {
			return this.getJointAngle() < Math.toRadians(shoulderLimits[1]);
		}
	}

	public static class ElbowJoint implements ArmJoint {
		private final CANSparkMax motor = new CANSparkMax(Arm.ELBOW_JOINT_ID, MotorType.kBrushless);
		private final RelativeEncoder encoder = motor.getEncoder();

		private final SparkMaxLimitSwitch switchForward = motor.getForwardLimitSwitch(Type.kNormallyOpen);

		private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(9);

		private final ProfiledPIDController controller = new ProfiledPIDController(0.7, 0.05, 0,
				new TrapezoidProfile.Constraints(MAX_VELOCITY_ELBOW, MAX_ACCELERATION));
		private final ArmFeedforward feedForward = new ArmFeedforward(0, -0.4, 0, 0);
		private MechanismLigament2d simulator;
		private MechanismLigament2d real;

		public ElbowJoint() {
			motor.setIdleMode(IdleMode.kBrake);
			motor.setInverted(false);
			// motor.setSmartCurrentLimit(30);

			encoder.setPositionConversionFactor(2 * Math.PI * (1 / Arm.GEAR_RATIO_ELBOW));
			controller.setTolerance(Math.toRadians(2));

		}

		@Override
		public void setSpeed(double speed) {
			motor.set(speed);
		}

		@Override
		public double getJointAngle() {

			return MathUtil.angleModulus(Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition())
					+ Math.toRadians(ELBOW_START_ANGLE));
		}

		@Override
		public double getEncoderPosition() {
			return encoder.getPosition();
		}

		@Override
		public boolean atHardLimit() {
			return switchForward.isPressed();
		}

		@Override
		public boolean atSoftForwardLimit() {
			return false;
		}

		@Override
		public boolean atSoftReverseLimit() {
			return this.getJointAngle() < Math.toRadians(elbowLimits[1]);
		}
	}

	public static class WristJoint implements ArmJoint {
		private final CANSparkMax motor = new CANSparkMax(Arm.CLAW_JOINT_ID, MotorType.kBrushless);
		private final RelativeEncoder encoder = motor.getEncoder();

		private final SparkMaxLimitSwitch switchReverse = motor.getReverseLimitSwitch(Type.kNormallyOpen);

		private final ProfiledPIDController controller = new ProfiledPIDController(0.7, 0.05, 0,
				new TrapezoidProfile.Constraints(MAX_VELOCITY_WRIST, MAX_ACCELERATION));
		private final ArmFeedforward feedForward = new ArmFeedforward(0.01, 0, 0, 0);
		private MechanismLigament2d simulator;
		private MechanismLigament2d real;

		public WristJoint() {
			motor.setIdleMode(IdleMode.kBrake);
			motor.setInverted(true);
			motor.setSmartCurrentLimit(30);

			encoder.setPositionConversionFactor(2 * Math.PI * (1 / Arm.GEAR_RATIO_WRIST));
			controller.setTolerance(Math.toRadians(2));

		}

		@Override
		public void setSpeed(double speed) {
			// motor.set(speed);
			motor.set(0);
		}

		@Override
		public double getJointAngle() {
			final double WRIST_ANGLE_ERROR = Math.toRadians(3);

			return encoder.getPosition() + WRIST_ANGLE_ERROR;
		}

		@Override
		public double getEncoderPosition() {
			return encoder.getPosition();
		}

		@Override
		public boolean atHardLimit() {
			return switchReverse.isPressed();
		}

		@Override
		public boolean atSoftForwardLimit() {
			return this.getJointAngle() > Math.toRadians(wristLimits[0]);
		}

		@Override
		public boolean atSoftReverseLimit() {
			return this.getJointAngle() < Math.toRadians(wristLimits[1]);
		}
	}

	public static ArmChain armChainHorizontal = new ArmChain();
	public static ArmChain armChainVertical = new ArmChain();
	public static ArmChain armChainVertical2 = new ArmChain();
	public FabrikController armKinematics = new FabrikController();

	public ShoulderJoint shoulderJoint = new ShoulderJoint();
	public ElbowJoint elbowJoint = new ElbowJoint();
	public WristJoint wristJoint = new WristJoint();

	public final double ENCODERS_PER_REV = wristJoint.encoder.getCountsPerRevolution();

	private final Mechanism2d armSimulation;
	public MechanismRoot2d effectorRoot;
	private MechanismLigament2d effectorPoint;

	private double shoulderReachAngle;
	private double elbowReachAngle;
	private double wristReachAngle;

	// forward limit, reverse limit
	public static float[] shoulderLimits = { 50, -19.5f };
	public static float[] elbowLimits = { 170, 30 };
	public static float[] wristLimits = { 135, -135 };

	// Home position angles for each joint
	public static final double SHOULDER_START_ANGLE = -10.5;
	public static final double ELBOW_START_ANGLE = 160.0;
	public static final double WRIST_START_ANGLE = -100.0;

	public ArmSubsystem() {
		// Initialize Inverse Kinematics with constant values
		if (manipulator instanceof ClawSubsystem) {
			this.armChainHorizontal.setArmLength(62f)
					.setSegmentLengthRatio(0, 31 / 62f)
					.setSegmentLengthRatio(1, 31 / 62f)
					.setSegmentLengths()
					.setAngleConstraint(0, shoulderLimits[0], -shoulderLimits[1])
					.setAngleConstraint(1, elbowLimits[0], -elbowLimits[1])
					.setSegmentInitialDirection(0, (float) Math.toRadians(90))
					.setSegmentInitialDirection(1, (float) Math.toRadians(0))
					.initializeChain()
					.addGloballyConstrainedGripper((float) Math.toRadians(0), 4f)
					.setSolveDistanceThreshold(1f)
					.setMaxIterationAttempts(5000);

			this.armChainVertical.setArmLength(62f)
					.setSegmentLengthRatio(0, 31 / 62f)
					.setSegmentLengthRatio(1, 31 / 62f)
					.setSegmentLengths()
					.setAngleConstraint(0, shoulderLimits[0], -shoulderLimits[1])
					.setAngleConstraint(1, elbowLimits[0], -elbowLimits[1])
					.setSegmentInitialDirection(0, (float) Math.toRadians(90))
					.setSegmentInitialDirection(1, (float) Math.toRadians(0))
					.initializeChain()
					.addGloballyConstrainedGripper((float) Math.toRadians(30), 4f)
					.setSolveDistanceThreshold(1f)
					.setMaxIterationAttempts(5000);
			this.armChainVertical2.setArmLength(62f)
					.setSegmentLengthRatio(0, 31 / 62f)
					.setSegmentLengthRatio(1, 31 / 62f)
					.setSegmentLengths()
					.setAngleConstraint(0, shoulderLimits[0], -shoulderLimits[1])
					.setAngleConstraint(1, elbowLimits[0], -elbowLimits[1])
					.setSegmentInitialDirection(0, (float) Math.toRadians(90))
					.setSegmentInitialDirection(1, (float) Math.toRadians(0))
					.initializeChain()
					.addGloballyConstrainedGripper((float) Math.toRadians(-30), 4f)
					.setSolveDistanceThreshold(1f)
					.setMaxIterationAttempts(5000);
		} else if (manipulator instanceof IntakeSubsystem) {
			this.armChainHorizontal.setArmLength(62f)
					.setSegmentLengthRatio(0, 31 / 62f)
					.setSegmentLengthRatio(1, 31 / 62f)
					.setSegmentLengths()
					.setAngleConstraint(0, shoulderLimits[0], -shoulderLimits[1])
					.setAngleConstraint(1, elbowLimits[0], -elbowLimits[1])
					.setSegmentInitialDirection(0, (float) Math.toRadians(90))
					.setSegmentInitialDirection(1, (float) Math.toRadians(0))
					.initializeChain()
					.addGloballyConstrainedGripper((float) Math.toRadians(-45), 8f)
					.setSolveDistanceThreshold(1f)
					.setMaxIterationAttempts(5000);

			this.armChainVertical.setArmLength(62f)
					.setSegmentLengthRatio(0, 31 / 62f)
					.setSegmentLengthRatio(1, 31 / 62f)
					.setSegmentLengths()
					.setAngleConstraint(0, shoulderLimits[0], -shoulderLimits[1])
					.setAngleConstraint(1, elbowLimits[0], -elbowLimits[1])
					.setSegmentInitialDirection(0, (float) Math.toRadians(90))
					.setSegmentInitialDirection(1, (float) Math.toRadians(0))
					.initializeChain()
					.addGloballyConstrainedGripper((float) Math.toRadians(30), 8f)
					.setSolveDistanceThreshold(1f)
					.setMaxIterationAttempts(5000);
			this.armChainVertical2.setArmLength(62f)
					.setSegmentLengthRatio(0, 31 / 62f)
					.setSegmentLengthRatio(1, 31 / 62f)
					.setSegmentLengths()
					.setAngleConstraint(0, shoulderLimits[0], -shoulderLimits[1])
					.setAngleConstraint(1, elbowLimits[0], -elbowLimits[1])
					.setSegmentInitialDirection(0, (float) Math.toRadians(90))
					.setSegmentInitialDirection(1, (float) Math.toRadians(0))
					.initializeChain()
					.addGloballyConstrainedGripper((float) Math.toRadians(-120), 8f)
					.setSolveDistanceThreshold(1f)
					.setMaxIterationAttempts(5000);

		} else {
		}

		// Create arm simulation components

		armSimulation = new Mechanism2d(300 / Arm.SIMULATION_SCALE, 300 / Arm.SIMULATION_SCALE);

		// Append arm joints Joints

		MechanismRoot2d armRoot = armSimulation.getRoot(
				"Arm",
				(Arm.SIMULATION_OFFSET + 150) / Arm.SIMULATION_SCALE,
				(Arm.SIMULATION_OFFSET) / Arm.SIMULATION_SCALE);

		shoulderJoint.simulator = armRoot.append(
				new MechanismLigament2d(
						"shoulder",
						(double) armChainHorizontal.getSegmentLength(0) / Arm.SIMULATION_SCALE,
						0,
						5.0,
						new Color8Bit(255, 0, 0)));

		elbowJoint.simulator = shoulderJoint.simulator.append(
				new MechanismLigament2d(
						"elbow",
						(double) (armChainHorizontal.getSegmentLength(1)) / Arm.SIMULATION_SCALE,
						0.0,
						5.0,
						new Color8Bit(0, 255, 0)));

		wristJoint.simulator = elbowJoint.simulator.append(
				new MechanismLigament2d(
						"wrist",
						(double) 4 / Arm.SIMULATION_SCALE,
						0,
						5.0,
						new Color8Bit(0, 0, 255)));

		MechanismRoot2d armRootReal = armSimulation.getRoot(
				"RealArm",
				(Arm.SIMULATION_OFFSET + 150) / Arm.SIMULATION_SCALE,
				(Arm.SIMULATION_OFFSET) / Arm.SIMULATION_SCALE);

		shoulderJoint.real = armRootReal.append(
				new MechanismLigament2d(
						"shoulderReal",
						(double) armChainHorizontal.getSegmentLength(0) / Arm.SIMULATION_SCALE,
						0,
						5.0,
						new Color8Bit(255, 0, 255)));

		elbowJoint.real = shoulderJoint.real.append(
				new MechanismLigament2d(
						"elbowReal",
						(double) (armChainHorizontal.getSegmentLength(1)) / Arm.SIMULATION_SCALE,
						0.0,
						5.0,
						new Color8Bit(255, 255, 0)));

		wristJoint.real = elbowJoint.real.append(
				new MechanismLigament2d(
						"wristReal",
						(double) 4 / Arm.SIMULATION_SCALE,
						0,
						5.0,
						new Color8Bit(0, 255, 255)));

		// Append effector points

		effectorRoot = armSimulation.getRoot(
				"End Effector",
				(Arm.SIMULATION_OFFSET + 150) / Arm.SIMULATION_SCALE + cartesianStorage.getX(),
				(Arm.SIMULATION_OFFSET) / Arm.SIMULATION_SCALE + cartesianStorage.getZ());

		effectorPoint = effectorRoot.append(
				new MechanismLigament2d(
						"effector",
						1.0 / Arm.SIMULATION_SCALE,
						0.0,
						5.0,
						new Color8Bit(255, 0, 0)));

		// Set default reach angles
		shoulderReachAngle = armChainHorizontal.getIKAnglesDegrees().get(0);
		elbowReachAngle = shoulderReachAngle - armChainHorizontal.getIKAnglesDegrees().get(1);
	}

	/**
	 * Reach a point from a translation
	 */
	// public void reach(Translation3d translation) {
	// 	armKinematics.solveNewTarget(translation);

	// 	if (Robot.operatorController.getRightTriggerAxis() < 0.9) {
	// 		reach(armKinematics.getAnglesRadians(0));
	// 	} else {
	// 		reach(armKinematics.getAnglesRadians(1));
	// 	}
	// }

	/**
	 * Update the embeded anlges from a 3d point and reach for that point
	 */
	public List<Double> reachEmbedded(Translation3d translation) {
		if (Robot.operatorController.getRightTriggerAxis() > 0.9) {
			armChainVertical.updateEmbedded((float) translation.getX(), (float) translation.getZ());
			armChainVertical.solveForEmbedded();
			return reach(armChainVertical.getIKAnglesRadians());
	
		}
		else if ((isVerticalChain2)) {
			armChainVertical2.updateEmbedded((float) translation.getX(), (float) translation.getZ());
			armChainVertical2.solveForEmbedded();
			return reach(armChainVertical2.getIKAnglesRadians());
		}
		else {
			

			armChainHorizontal.updateEmbedded((float) translation.getX(), (float) translation.getZ());
			armChainHorizontal.solveForEmbedded();
			return reach(armChainHorizontal.getIKAnglesRadians());
		}
	}

	/**
	 * Reach for a list of raw desired angles
	 * <br>
	 * <br>
	 * Order is [shoulder, elbow, wrist]
	 */
	public List<Double> reach(List<Double> desiredRadianAngles) {
		// Pull angles out of list
		double desiredShoulderAngle = desiredRadianAngles.get(0);
		double desiredElbowAngle = desiredRadianAngles.get(1);
		double desiredWristAngle = desiredRadianAngles.get(2);

		shoulderReachAngle = desiredShoulderAngle;
		elbowReachAngle = desiredElbowAngle;
		wristReachAngle = desiredWristAngle;

		// Round to nearest degree for shuffleboard display
		shoulderAngleEntry.setDouble(Math.round(Math.toDegrees(desiredShoulderAngle)));
		elbowAngleEntry.setDouble(Math.round(Math.toDegrees(desiredElbowAngle)));
		wristAngleEntry.setDouble(Math.round(Math.toDegrees(desiredWristAngle)));

		// Compute feedforward
		// TODO: Recompute the angles to fit the inputs wanted by the feedforward
		// controller (relative to horizontal)

		// elbowJoint.controller.setP(((Math.PI / 2 - (desiredShoulderAngle +
		// desiredElbowAngle)) + Math.PI / 2) * 0.5);
		// System.out.println(elbowJoint.controller.getP());

		double shoulderFeedForward = shoulderJoint.feedForward.calculate(Math.PI / 2 - desiredShoulderAngle, 0);
		double elbowFeedForward = elbowJoint.feedForward
				.calculate(Math.PI / 2 - (desiredShoulderAngle + desiredElbowAngle), 0);
		double wristFeedForward = wristJoint.feedForward
				.calculate(Math.PI / 2 - (desiredShoulderAngle + desiredElbowAngle + desiredWristAngle), 0);

		// Compute feedback
		double shoulderFeedback = shoulderJoint.controller.calculate(
				shoulderJoint.getJointAngle(),
				desiredShoulderAngle);
		double elbowFeedback = elbowJoint.controller.calculate(
				elbowJoint.getJointAngle(),
				desiredElbowAngle);
		double wristFeedback = wristJoint.controller.calculate(
				wristJoint.getJointAngle(),
				desiredWristAngle);

		// Compute joint speeds based on feedback and feedforward while limiting
		// movement based on hard and soft limits

		final double BOUNCE_FORCE = -0.125;

		double shoulderSpeed = shoulderFeedback;
		double elbowSpeed = elbowFeedback;
		double wristSpeed = wristFeedback;

		boolean forwardShoulderLimit = Math.signum(shoulderSpeed) > 0
				&& shoulderJoint.atSoftForwardLimit();
		boolean reverseShoulderLimit = Math.signum(shoulderSpeed) < 0
				&& shoulderJoint.atSoftReverseLimit();

		// if (Math.signum(shoulderSpeed) < 0 && shoulderJoint.atHardLimit()) {
		// shoulderSpeed = 0;
		// } else if (forwardShoulderLimit || reverseShoulderLimit) {
		// shoulderSpeed *= BOUNCE_FORCE;
		// }

		boolean forwardElbowLimit = Math.signum(elbowSpeed) > 0
				&& elbowJoint.atSoftForwardLimit();
		boolean reverseElbowLimit = Math.signum(elbowSpeed) < 0
				&& elbowJoint.atSoftReverseLimit();

		// if (Math.signum(elbowSpeed) > 0 && elbowJoint.atHardLimit()) {
		// elbowSpeed = 0;
		// System.out.println("Elbow Hard limit reached and going forward");
		// } else
		// if (forwardElbowLimit || reverseElbowLimit) {
		// elbowSpeed *= BOUNCE_FORCE;
		// System.out.println("elbow soft limit reached");
		// }

		boolean forwardWristLimit = Math.signum(wristSpeed) > 0
				&& (wristJoint.atSoftForwardLimit());
		boolean reverseWristLimit = Math.signum(wristSpeed) < 0
				&& (wristJoint.atHardLimit() || wristJoint.atSoftReverseLimit());

		// if (forwardWristLimit || reverseWristLimit) {
		// wristSpeed *= BOUNCE_FORCE;
		// }

		// Set motor speeds
		// if (Math.abs(Math.toDegrees(elbowJoint.getJointAngle() - desiredRadianAngles.get(1))) < 20) {
		shoulderJoint.setSpeed(shoulderSpeed);
		// }
		elbowJoint.setSpeed(elbowSpeed);


		// REMOVE WRIST JOINT DUE TO MECHANICAL ISSUES

		// wristJoint.setSpeed(wristSpeed);

		// Update shuffleboard
		shoulderSpeedEntry.setDouble(shoulderSpeed);
		elbowSpeedEntry.setDouble(elbowSpeed);
		wristSpeedEntry.setDouble(wristSpeed);

		return desiredRadianAngles;
	}

	/**
	 * Calculates the difference between desired end effector location and its
	 * current location
	 *
	 * @return Distance in inches within the X-Z plane
	 */
	public float getPositionError() {
		return armChainHorizontal.getIKPositionError();
	}

	public void move(double shoulderSpeed, double elbowSpeed, double wristSpeed) {
		/* Shoulder */

		boolean forwardShoulderLimit = Math.signum(shoulderSpeed) > 0
				&& shoulderJoint.atSoftForwardLimit();
		boolean reverseShoulderLimit = Math.signum(shoulderSpeed) < 0
				&& (shoulderJoint.atHardLimit() || shoulderJoint.atSoftReverseLimit());

		if (!forwardShoulderLimit && !reverseShoulderLimit) {
			shoulderJoint.setSpeed(shoulderSpeed);
		}

		/* Elbow */

		boolean forwardElbowLimit = Math.signum(elbowSpeed) > 0
				&& (elbowJoint.atHardLimit() || elbowJoint.atSoftForwardLimit());
		boolean reverseElbowLimit = Math.signum(elbowSpeed) < 0
				&& (elbowJoint.atSoftReverseLimit());

		if (!forwardElbowLimit && !reverseElbowLimit) {
			elbowJoint.setSpeed(elbowSpeed);
		}

		/* Wrist */

		boolean forwardWristLimit = Math.signum(wristSpeed) > 0
				&& (wristJoint.atSoftForwardLimit());
		boolean reverseWristLimit = Math.signum(wristSpeed) < 0
				&& (wristJoint.atHardLimit() || wristJoint.atSoftReverseLimit());

		if (!forwardWristLimit && !reverseWristLimit) {

			//REMOVE WRIST MOVEMENT
			// wristJoint.setSpeed(wristSpeed);
		}
	}

	/**
	 * Sets the relative encoders to use the desired angles
	 */
	public void setEncodersDegrees(double shoulderAngleDegrees, double elbowAngleDegrees, double wristAngleDegrees) {
		// Convert to radians
		double shoulderAngleRadians = Math.toRadians(shoulderAngleDegrees);
		double elbowAngleRadians = Math.toRadians(elbowAngleDegrees);
		double wristAngleRadians = Math.toRadians(wristAngleDegrees);

		// Set encoder positions
		shoulderJoint.leftEncoder.setPosition(shoulderAngleRadians);
		shoulderJoint.rightEncoder.setPosition(shoulderAngleRadians);

		elbowJoint.encoder.setPosition(elbowAngleRadians);

		wristJoint.encoder.setPosition(wristAngleRadians);

		// Reset PID controllers
		shoulderJoint.controller.reset(shoulderAngleRadians);
		elbowJoint.controller.reset(elbowAngleRadians);
		wristJoint.controller.reset(wristAngleRadians);
	}

	public void resetEncodersToHomePosition() {
		// Reset the arm relative encoders to known angles
		this.setEncodersDegrees(SHOULDER_START_ANGLE, ELBOW_START_ANGLE, WRIST_START_ANGLE);
	}

	/**
	 * Sets each joint motor to rotate towards the home position unless it is at its
	 * hard limit
	 */
	public void homeJoints() {
		// Move shoulder
		if (!shoulderJoint.atHardLimit()) {
			shoulderJoint.setSpeed(-0.2);
		} else {
			shoulderJoint.setSpeed(0);
		}

		// Move elbow
		if (!elbowJoint.atHardLimit()) {
			elbowJoint.setSpeed(0.2);
		} else {
			elbowJoint.setSpeed(0);
		}

		// Move wrist

		//REMOVE WRIST JOINT DUE TO MECHANICAL ERROR

		// if (!wristJoint.atHardLimit()) {
		// 	wristJoint.setSpeed(-0.3);
		// } else {
		// 	wristJoint.setSpeed(0);
		// }
	}

	/**
	 * Sets each joint motor to rotate towards the home position unless it is at its
	 * 
	 * @param shoulderSpeed speed of shoulder
	 * @param elbowSpeed    speed of elbow
	 * @param wristSpeed    speed of wrist
	 *                      hard limit
	 */
	public void homeJoints(double shoulderSpeed, double elbowSpeed, double wristSpeed) {
		// Move shoulder
		shoulderJoint.setSpeed(!shoulderJoint.atHardLimit() ? -Math.abs(shoulderSpeed) : 0);

		// Move elbow
		elbowJoint.setSpeed(!elbowJoint.atHardLimit() ? Math.abs(elbowSpeed) : 0);

		// Move wrist
		// REMOVE WRIST MOVEMENT DUE TO MECHANICAL ERROR
		// wristJoint.setSpeed(!wristJoint.atHardLimit() ? -Math.abs(wristSpeed) : 0);
	}

	public void stopMotors() {
		shoulderJoint.setSpeed(0);
		elbowJoint.setSpeed(0);
		wristJoint.setSpeed(0);
	}

	/**
	 * Checks to see if all the hard limits are pressed
	 */
	public boolean isInHomePosition() {
		return shoulderJoint.atHardLimit()
				&& elbowJoint.atHardLimit()
				&& wristJoint.atHardLimit();
	}

	@Override
	public void periodic() {
		// Update the sim angles if reach hasnt been called?
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

		if (shoulderSimAngle != this.shoulderReachAngle
				|| elbowSimAngle != this.elbowReachAngle
				|| wristSimAngle != this.wristReachAngle) {
			this.shoulderReachAngle = shoulderSimAngle;
			this.elbowReachAngle = elbowSimAngle;
			this.wristReachAngle = wristSimAngle;
		}

		shoulderJoint.simulator.setAngle(this.shoulderReachAngle);
		elbowJoint.simulator.setAngle(this.elbowReachAngle);
		wristJoint.simulator.setAngle(this.wristReachAngle);

		shoulderJoint.real.setAngle(-Math.toDegrees(this.shoulderJoint.getJointAngle()) + 90);
		elbowJoint.real.setAngle(-Math.toDegrees(this.elbowJoint.getJointAngle()));
		wristJoint.real.setAngle(-Math.toDegrees(this.wristJoint.getJointAngle()));

		effectorXEntry.setDouble(cartesianStorage.getX());
		effectorYEntry.setDouble(cartesianStorage.getZ());

		shoulderEncoderEntry.setDouble(shoulderJoint.getEncoderPosition());
		elbowEncoderEntry.setDouble(elbowJoint.getEncoderPosition());
		wristEncoderEntry.setDouble(wristJoint.getEncoderPosition());

		shoulderSwitchEntry.setBoolean(shoulderJoint.atHardLimit());
		elbowSwitchEntry.setBoolean(elbowJoint.atHardLimit());
		wristSwitchEntry.setBoolean(wristJoint.atHardLimit());

		shoulderAbsoluteAngleEntry.setDouble(Math.toDegrees(shoulderJoint.getJointAngle()));
		elbowAbsoluteAngleEntry.setDouble(Math.toDegrees(elbowJoint.getJointAngle()));
		wristAbsoluteAngleEntry.setDouble(Math.toDegrees(wristJoint.getJointAngle()));

		shoulderSoftLimitEntry.setBoolean(shoulderJoint.atSoftLimit());
		elbowSoftLimitEntry.setBoolean(elbowJoint.atSoftLimit());
		wristSoftLimitEntry.setBoolean(wristJoint.atSoftLimit());

		shoulderSoftForwardLimitEntry.setBoolean(shoulderJoint.atSoftForwardLimit());
		elbowSoftForwardLimitEntry.setBoolean(elbowJoint.atSoftForwardLimit());
		wristSoftForwardLimitEntry.setBoolean(wristJoint.atSoftForwardLimit());

		shoulderSoftReverseLimitEntry.setBoolean(shoulderJoint.atSoftReverseLimit());
		elbowSoftReverseLimitEntry.setBoolean(elbowJoint.atSoftReverseLimit());
		wristSoftReverseLimitEntry.setBoolean(wristJoint.atSoftReverseLimit());

		shoulderEncoderErrorEntry.setDouble(shoulderJoint.getJointAngle() - shoulderJoint.getEncoderPosition());
		elbowEncoderErrorEntry.setDouble(elbowJoint.getJointAngle() - elbowJoint.getEncoderPosition());
		wristEncoderErrorEntry.setDouble(wristJoint.getJointAngle() - wristJoint.getEncoderPosition());

		shoulderAbsoluteAngleFloorEntry.setDouble(90 - Math.toDegrees(shoulderJoint.getJointAngle()));
		elbowAbsoluteAngleFloorEntry.setDouble(
				90 - (Math.toDegrees(shoulderJoint.getJointAngle()) + Math.toDegrees(elbowJoint.getJointAngle())));
		wristAbsoluteAngleFloorEntry.setDouble(90 - (Math.toDegrees(shoulderJoint.getJointAngle())
				+ Math.toDegrees(elbowJoint.getJointAngle()) + Math.toDegrees(wristJoint.getJointAngle())));

		Logger.getInstance().recordOutput("MyMechanism", this.armSimulation);
		Logger.getInstance().recordOutput("Raw Acceleration", Robot.navX.getRawAccelX());
	}
}