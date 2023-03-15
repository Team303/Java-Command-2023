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
import com.revrobotics.CANSparkMax.IdleMode;
import com.team303.robot.RobotMap.Arm;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import static com.team303.robot.commands.arm.DefaultIKControlCommand.cartesianStorage;
import com.team303.robot.RobotMap.Arm;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class ArmSubsystem extends SubsystemBase {

	public static final ShuffleboardTab ARM_TAB = Shuffleboard.getTab("Arm");
	public static final NetworkTable armNetwork = NetworkTableInstance.getDefault().getTable("arm");
	public static final GenericEntry shoulderAngle = ARM_TAB.add("shoulderAngle", 0).getEntry();
	public static final GenericEntry jointAngle = ARM_TAB.add("jointAngle", 0).getEntry();
	public static final GenericEntry clawAngle = ARM_TAB.add("clawAngle", 0).getEntry();
	public static final GenericEntry effectorX = ARM_TAB.add("effectorX", 0).getEntry();
	public static final GenericEntry effectorY = ARM_TAB.add("effectorY", 0).getEntry();
	public static final GenericEntry shoulderEncoderTab = ARM_TAB.add("shoulderEnc", 0).getEntry();
	public static final GenericEntry elbowEncoderTab = ARM_TAB.add("elbowEnc", 0).getEntry();
	public static final GenericEntry clawEncoderTab = ARM_TAB.add("clawEnc", 0).getEntry();
    public static final GenericEntry shoulderAngleAbsolute = ARM_TAB.add("shoulderAngleAbsolute", 0).getEntry();
	public static final GenericEntry jointAngleAbsolute = ARM_TAB.add("jointAngleAbsolute", 0).getEntry();
	public static final GenericEntry clawAngleAbsoludte = ARM_TAB.add("clawAngleAbsolute", 0).getEntry();
    public static final GenericEntry effectorAngle = ARM_TAB.add("effector angle", 0).getEntry();

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
		private final CANSparkMax shoulderMotor1 = new CANSparkMax(Arm.SHOULDER_JOINT_ID1, MotorType.kBrushless);
		private final CANSparkMax shoulderMotor2 = new CANSparkMax(Arm.SHOULDER_JOINT_ID2, MotorType.kBrushless);
		public final RelativeEncoder shoulderEncoder1 = shoulderMotor1.getEncoder();
		public final RelativeEncoder shoulderEncoder2 = shoulderMotor2.getEncoder();

		public final SparkMaxLimitSwitch shoulderSwitch1;
		public final SparkMaxLimitSwitch shoulderSwitch2;

		public ShoulderJoint() {
			shoulderMotor1.setIdleMode(IdleMode.kBrake);
			shoulderMotor2.setIdleMode(IdleMode.kBrake);

			shoulderSwitch1 = shoulderMotor1.getReverseLimitSwitch(Type.kNormallyClosed);
			shoulderSwitch2 = shoulderMotor2.getReverseLimitSwitch(Type.kNormallyClosed);

			shoulderMotor1.setInverted(false);
			shoulderMotor2.setInverted(true);

			// shoulderEncoder1.setInverted(false);
			// shoulderEncoder2.setInverted(true);
		}

		public final void setMotors(double speed) {
			shoulderMotor1.set(speed);
			shoulderMotor2.set(speed);
		}

		public final ProfiledPIDController shoulderControl = new ProfiledPIDController(0.01, 0, 0,
				new TrapezoidProfile.Constraints(CIRCUMFERENCE * VELOCITY, CIRCUMFERENCE * ACCELERATION));
		private final ArmFeedforward m_shoulderFeedForward = new ArmFeedforward(0.01, 0, 0, 0);
		private MechanismLigament2d shoulderSimulator;

	}

	public class ElbowJoint {
		public final CANSparkMax elbowMotor = new CANSparkMax(Arm.ELBOW_JOINT_ID, MotorType.kBrushless);
		public final SparkMaxLimitSwitch elbowSwitch;
		public final RelativeEncoder elbowEncoder = elbowMotor.getEncoder();

		public ElbowJoint() {
			elbowSwitch = elbowMotor.getReverseLimitSwitch(Type.kNormallyClosed);
			elbowMotor.setIdleMode(IdleMode.kBrake);
			elbowMotor.setInverted(false);
			// elbowEncoder.setInverted(false);
		}

		public ProfiledPIDController elbowControl = new ProfiledPIDController(0.01, 0, 0,
				new TrapezoidProfile.Constraints(CIRCUMFERENCE * VELOCITY, CIRCUMFERENCE * ACCELERATION));
		private final ArmFeedforward m_elbowFeedForward = new ArmFeedforward(0.01, 0, 0, 0);
		private MechanismLigament2d elbowSimulator;
	}

	public class ClawJoint {
		private final CANSparkMax clawMotor = new CANSparkMax(Arm.CLAW_JOINT_ID, MotorType.kBrushless);
		private final SparkMaxLimitSwitch clawSwitch;
		private final RelativeEncoder clawEncoder = clawMotor.getEncoder();

		public ClawJoint() {
			clawSwitch = clawMotor.getReverseLimitSwitch(Type.kNormallyClosed);
			clawMotor.setIdleMode(IdleMode.kBrake);
			clawMotor.setInverted(false);
			// clawEncoder.setInverted(false);
		}

		public ProfiledPIDController clawControl = new ProfiledPIDController(0.01, 0, 0,
				new TrapezoidProfile.Constraints(CIRCUMFERENCE * VELOCITY, CIRCUMFERENCE * ACCELERATION));
		private final ArmFeedforward m_clawFeedForward = new ArmFeedforward(0.01, 0, 0, 0);
		private MechanismLigament2d clawSimulator;
	}

	public static FabrikController armKinematics = new FabrikController();
	
	public ShoulderJoint shoulderJoint = new ShoulderJoint();
	public ElbowJoint elbowJoint = new ElbowJoint();
	public ClawJoint clawJoint = new ClawJoint();
	
	private Mechanism2d armSimulation;
	public MechanismRoot2d effectorRoot;
	private MechanismLigament2d effectorPoint;
	
	private double storedShoulderAngle; 
	private double storedElbowAngle;
	private double storedClawAngle ;

	//left limit, right limit
	public float[] shoulderLimits = {-20, 20};
	public float[] elbowLimits = {-170, 170};
	public float[] wristLimits = {-135, 135};

	private final double shoulderStartAngle;
	private final double elbowStartAngle;
	// private final double clawStartAngle;

	public ArmSubsystem() {

		SmartDashboard.putNumber("Neo counts per revolution", shoulderJoint.shoulderEncoder1.getCountsPerRevolution());

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
		// shoulderJoint.shoulderEncoder1.setPositionConversionFactor(Arm.GEAR_RATIO_SHOULDER);
		// elbowJoint.elbowEncoder.setPositionConversionFactor(Arm.GEAR_RATIO_ELBOW);
		// clawJoint.clawEncoder.setPositionConversionFactor(Arm.GEAR_RATIO_CLAW);

		armSimulation = new Mechanism2d(300/Arm.SIMULATION_SCALE, 300/Arm.SIMULATION_SCALE);
		MechanismRoot2d armRoot = armSimulation.getRoot("Arm", (Arm.SIMULATION_OFFSET + 150)/Arm.SIMULATION_SCALE, (Arm.SIMULATION_OFFSET)/Arm.SIMULATION_SCALE);
		shoulderJoint.shoulderSimulator = armRoot.append(new MechanismLigament2d("shoulder",
				(double) armKinematics.getSegmentLength(0)/Arm.SIMULATION_SCALE, 0, 5.0, new Color8Bit(255, 0, 0)));
		elbowJoint.elbowSimulator = shoulderJoint.shoulderSimulator.append(new MechanismLigament2d("elbow",
				(double) armKinematics.getSegmentLength(1)/Arm.SIMULATION_SCALE, 0.0, 5.0, new Color8Bit(0, 255, 0)));
		// clawJoint.clawSimulator = elbowJoint.elbowSimulator.append(new MechanismLigament2d("claw",
		// 		(double) armKinematics.getSegmentLength(2)/Arm.SIMULATION_SCALE, 0, 5.0, new Color8Bit(0, 0, 255)));
		effectorRoot = armSimulation.getRoot("End Effector", (Arm.SIMULATION_OFFSET + 150)/Arm.SIMULATION_SCALE+cartesianStorage.getX(),Arm.SIMULATION_OFFSET/Arm.SIMULATION_SCALE+cartesianStorage.getZ());

		effectorPoint = effectorRoot.append(new MechanismLigament2d("effector",1.0/Arm.SIMULATION_SCALE,0.0,5.0,new Color8Bit(255,0,0)));
		storedShoulderAngle = -90+armKinematics.getIKAnglesDegrees().get(0);
		storedElbowAngle = storedShoulderAngle-armKinematics.getIKAnglesDegrees().get(1);
		// storedClawAngle = storedElbowAngle+armKinematics.getIKAnglesDegrees().get(2);
		SmartDashboard.putNumber("Counts per rev", shoulderJoint.shoulderEncoder1.getCountsPerRevolution());
		shoulderStartAngle = (Math.toRadians(Math.round(-10)) / (Math.PI * 2)) * shoulderJoint.shoulderEncoder1.getCountsPerRevolution() * Arm.GEAR_RATIO_SHOULDER;
		elbowStartAngle = (Math.toRadians(Math.round(170.0)) / (Math.PI * 2)) * elbowJoint.elbowEncoder.getCountsPerRevolution() * Arm.GEAR_RATIO_ELBOW;
		// clawStartAngle = (Math.toRadians(Math.round(-135.0)) / (Math.PI * 2)) * clawJoint.clawEncoder.getCountsPerRevolution() * Arm.GEAR_RATIO_CLAW;

		// setEncoders(shoulderStartAngle,elbowStartAngle,0);
		System.out.println(shoulderStartAngle + " " + elbowStartAngle + " " + 0);
		// reach(List.of(0.0,170.0,135.0));	
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

		shoulderAngle.setDouble(Math.toDegrees(desiredRadianAngles.get(0)));
		jointAngle.setDouble(Math.toDegrees(desiredRadianAngles.get(1)));
		// clawAngle.setDouble(Math.toDegrees(desiredRadianAngles.get(2)));

		double shoulderEncoders = Math.round((desiredRadianAngles.get(0))/ (Math.PI * 2) * shoulderJoint.shoulderEncoder1.getCountsPerRevolution()) * Arm.GEAR_RATIO_SHOULDER;
		double elbowEncoders = Math.round((desiredRadianAngles.get(1)) / (Math.PI * 2) * shoulderJoint.shoulderEncoder1.getCountsPerRevolution()) * Arm.GEAR_RATIO_ELBOW;
		// double clawEncoders = Math.round((desiredRadianAngles.get(2)) / (Math.PI * 2) * shoulderJoint.shoulderEncoder1.getCountsPerRevolution()) * Arm.GEAR_RATIO_CLAW;

		shoulderEncoderTab.setDouble(shoulderEncoders);
		elbowEncoderTab.setDouble(elbowEncoders);
		// clawEncoderTab.setDouble(clawEncoders);
		// Gear ratio is 40 / 12 * 160  * ShoulderJoint.shoulderEncoder.getEncodersPer

		shoulderJoint.shoulderControl.setGoal(encodersToRadians(shoulderEncoders, Arm.GEAR_RATIO_SHOULDER) * CIRCUMFERENCE);
		elbowJoint.elbowControl.setGoal(encodersToRadians(elbowEncoders, Arm.GEAR_RATIO_ELBOW) * CIRCUMFERENCE);
		// clawJoint.clawControl.setGoal(clawEncoders);
		double shoulderFeedForward = shoulderJoint.m_shoulderFeedForward.calculate(
				shoulderJoint.shoulderControl.getGoal().position, shoulderJoint.shoulderControl.getGoal().velocity);
		double elbowFeedForward = elbowJoint.m_elbowFeedForward.calculate(elbowJoint.elbowControl.getGoal().position,
				elbowJoint.elbowControl.getGoal().velocity);
		// double clawFeedForward = clawJoint.m_clawFeedForward.calculate(clawJoint.clawControl.getGoal().position,
		// 		clawJoint.clawControl.getGoal().velocity);

		double shoulderFeedback = shoulderJoint.shoulderControl.calculate(
				encodersToRadians(shoulderJoint.shoulderEncoder1.getPosition(), Arm.GEAR_RATIO_SHOULDER) * CIRCUMFERENCE,
				shoulderJoint.shoulderControl.getGoal());
		double elbowFeedback = elbowJoint.elbowControl.calculate(
				encodersToRadians(elbowJoint.elbowEncoder.getPosition(), Arm.GEAR_RATIO_ELBOW) * CIRCUMFERENCE, elbowJoint.elbowControl.getGoal());
		// double clawFeedback = clawJoint.clawControl.calculate(
		// 		Units.rotationsToRadians(clawJoint.clawEncoder.getPosition()), clawJoint.clawControl.getGoal());

		SmartDashboard.putNumber("shoulderFeedForward", shoulderFeedForward);
		SmartDashboard.putNumber("shoulderFeeback", shoulderFeedback);

		if (shoulderJoint.shoulderEncoder1.getPosition() < degreesToEncoders(shoulderLimits[0], Arm.GEAR_RATIO_SHOULDER) ||
			shoulderJoint.shoulderEncoder1.getPosition() > degreesToEncoders(shoulderLimits[0], Arm.GEAR_RATIO_SHOULDER)) {
				shoulderJoint.setMotors(-(shoulderFeedForward + shoulderFeedback) * 0.02);
		} else {
			shoulderJoint.setMotors((shoulderFeedForward + shoulderFeedback) * 0.02);
		}

		if (elbowJoint.elbowEncoder.getPosition() < degreesToEncoders(elbowLimits[0], Arm.GEAR_RATIO_ELBOW) ||
			elbowJoint.elbowEncoder.getPosition() > degreesToEncoders(elbowLimits[0], Arm.GEAR_RATIO_ELBOW)) {
				elbowJoint.elbowMotor.set(-(elbowFeedForward + elbowFeedback) * 0.02);
		} else {
			elbowJoint.elbowMotor.set((elbowFeedForward + elbowFeedback) * 0.02);
		}

		// if (clawJoint.clawEncoder.getPosition() < degreesToEncoders(wristLimits[0], Arm.GEAR_RATIO_CLAW) ||
		// clawJoint.clawEncoder.getPosition() > degreesToEncoders(wristLimits[0], Arm.GEAR_RATIO_CLAW)) {
		// 	clawJoint.clawMotor.set(-(clawFeedForward + clawFeedback) * 0.02);
		// } else {
		// 	clawJoint.clawMotor.set((clawFeedForward + clawFeedback) * 0.02);
		// }
	}

	public void resetEncoders() {
		shoulderJoint.shoulderEncoder1.setPosition(0.0);
		elbowJoint.elbowEncoder.setPosition(0.0);
		clawJoint.clawEncoder.setPosition(0.0);
	}

	public void setEncoders(double shoulder, double elbow, double claw) {
		shoulderJoint.shoulderEncoder1.setPosition(shoulder);
		elbowJoint.elbowEncoder.setPosition(elbow);
		clawJoint.clawEncoder.setPosition(claw);
	}

	public double[] getEncoderPosition() {
		return new double[] { shoulderJoint.shoulderEncoder1.getPosition(), elbowJoint.elbowEncoder.getPosition(),
				clawJoint.clawEncoder.getPosition() };
	}

	public double[] getJointRevolutions() {
		return new double[] { shoulderJoint.shoulderEncoder1.getCountsPerRevolution(),
				elbowJoint.elbowEncoder.getCountsPerRevolution(), clawJoint.clawEncoder.getCountsPerRevolution() };
	}

	public float getError() {
		return armKinematics.getIKPositionError();
	}

	@Override
	public void periodic() {
		JOINT_ANGLES_PUB.set(JOINT_ANGLES_SUB.get(new double[] { 0.0, 0.0 }));
		JOINT_RPM_PUB.set(JOINT_RPM_SUB.get(new double[] { 0.0, 0.0 }));
		double shoulderSimAngle = 90-armKinematics.getIKAnglesDegrees().get(0);
		double elbowSimAngle =-armKinematics.getIKAnglesDegrees().get(1);
		// double clawSimAngle = -armKinematics.getIKAnglesDegrees().get(2);

		if (shoulderSimAngle != storedShoulderAngle || elbowSimAngle != storedElbowAngle /* || clawSimAngle != storedClawAngle */) {
			storedShoulderAngle = shoulderSimAngle;
			storedElbowAngle = elbowSimAngle;
			// storedClawAngle = clawSimAngle;
			// double absoluteElbow = storedShoulderAngle + storedElbowAngle;
			// double absoluteClaw = storedClawAngle + storedShoulderAngle + storedElbowAngle;
			// double targetAngle = Math.toDegrees(Math.atan2(cartesianStorage.getZ(), cartesianStorage.getX()));

			// shoulderAngleAbsolute.setDouble(storedShoulderAngle);
			// jointAngleAbsolute.setDouble(absoluteElbow);
			// clawAngleAbsolute.setDouble(absoluteClaw);
		}

		// if (shoulderJoint.shoulderSwitch2.isPressed() || shoulderJoint.shoulderSwitch1.isPressed()) {
		// 	CommandScheduler.getInstance().cancel(new DefaultIKControlCommand(true));
		// 	shoulderJoint.setMotors(0);
		// }
		// if (elbowJoint.elbowSwitch.isPressed()) {
		// 	CommandScheduler.getInstance().cancel(new DefaultIKControlCommand(true));
		// 	elbowJoint.elbowMotor.set(0);
		// }
		// if (clawJoint.clawSwitch.isPressed()) {
		// 	CommandScheduler.getInstance().cancel(new DefaultIKControlCommand(true));
		// 	clawJoint.clawMotor.set(0);
		// }

		shoulderJoint.shoulderSimulator.setAngle(storedShoulderAngle);
		elbowJoint.elbowSimulator.setAngle(storedElbowAngle);
		// clawJoint.clawSimulator.setAngle(storedClawAngle);
        effectorAngle.setDouble(Math.toDegrees(Math.atan2(cartesianStorage.getZ(), cartesianStorage.getX())));
		effectorX.setDouble(cartesianStorage.getX());
		effectorY.setDouble(cartesianStorage.getZ());
		Logger.getInstance().recordOutput("MyMechanism", this.armSimulation);
	} 
}
