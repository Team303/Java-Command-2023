package com.team303.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.team303.robot.util.GroundedDigitalInput;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.team303.robot.RobotMap.Claw;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClawSubsystem extends SubsystemBase {
	/* ShuffleBoard */
	public static final ShuffleboardTab CLIMBER_TAB = Shuffleboard.getTab("Climber");
	public static final double GEAR_RATIO_WRIST = 27;

	private final CANSparkMax clawMotor = new CANSparkMax(20, MotorType.kBrushless);
	private final CANSparkMax clawRollMotor = new CANSparkMax(21, MotorType.kBrushless);
	private final RelativeEncoder clawEncoder;
	private final RelativeEncoder clawRollEncoder;
	public final SparkMaxLimitSwitch  clawOuterLimit;
	private final DigitalInput ultrasonicSensor = new DigitalInput(Claw.ULTRASONIC_ID);

	public ClawSubsystem() {
		clawMotor.setInverted(true);
		clawMotor.setIdleMode(IdleMode.kBrake);

		clawRollMotor.setInverted(false);
		clawRollMotor.setIdleMode(IdleMode.kBrake);

		clawEncoder = clawMotor.getEncoder();
		clawRollEncoder = clawRollMotor.getEncoder();

		clawOuterLimit = clawMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    }

	//claw motor getter among us imposter mode
	public CANSparkMax getClawMotor() {
		return clawMotor;
	}

	//limits
	public boolean outerLimitReached() {
		return clawOuterLimit.isPressed();
	}

    public double getClawPosition() {
        return clawEncoder.getPosition();
    }

    public double getRotatePosition() {
        return clawRollEncoder.getPosition();
    }
	public boolean getUltrasonicDistance() {
		return ultrasonicSensor.get();
	}

	public void setRotateSpeed(double wrist) {
		clawMotor.set(wrist);
	}

	public void setClawSpeed(double claw) {
		clawMotor.set(claw);
	}

	public void rotate(double angle, double speed) {

        angle %= 360;
        
        if (angle < 0) {
            angle += 360;
        }

		double startPos = clawRollEncoder.getPosition() / GEAR_RATIO_WRIST;

		while (clawRollEncoder.getPosition() / GEAR_RATIO_WRIST < angle + startPos) {
			clawRollMotor.set(speed);
		}
	}
        
	public boolean getEncoderPos(double encoderLimit) {
		return clawEncoder.getPosition() == encoderLimit;
	}

	public boolean getRotate(double rotateLimit) {
		return clawRollEncoder.getPosition() == rotateLimit;
	}

	public void resetEncoders() {
		clawEncoder.setPosition(0.0);
		// rotateEncoder.setPosition(0.0);
	}
}