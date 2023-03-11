package com.team303.robot.subsystems;

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
import com.team303.robot.RobotMap.Claw;

public class ClawSubsystem extends SubsystemBase {
	/* ShuffleBoard */
	public static final ShuffleboardTab CLIMBER_TAB = Shuffleboard.getTab("Climber");

	private final CANSparkMax clawCanSparkMax = new CANSparkMax(20, MotorType.kBrushless);
	private final CANSparkMax rotateCANSparkMax = new CANSparkMax(21, MotorType.kBrushless);
	private final RelativeEncoder clawEncoder;
	private final SparkMaxAbsoluteEncoder rotateEncoder;
	public final GroundedDigitalInput outerLeft;
	public final GroundedDigitalInput outerRight;
	public final GroundedDigitalInput innerLeft;
	public final GroundedDigitalInput innerRight;
	private final AnalogPotentiometer ultrasonicSensor = new AnalogPotentiometer(Claw.ULTRASONIC_ID,300,50);

	public ClawSubsystem() {
		clawCanSparkMax.setInverted(true);
		clawCanSparkMax.setIdleMode(IdleMode.kBrake);
		clawEncoder = clawCanSparkMax.getEncoder();
		rotateEncoder = rotateCANSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
        rotateEncoder.setZeroOffset(0.0);
		// limit switches
		innerLeft = new GroundedDigitalInput(Claw.INNER_LEFT_INPUT);
		innerRight = new GroundedDigitalInput(Claw.INNER_RIGHT_INPUT);
		outerLeft = new GroundedDigitalInput(Claw.OUTER_LEFT_INPUT);
		outerRight = new GroundedDigitalInput(Claw.OUTER_RIGHT_INPUT);
    }

	//claw motor getter among us imposter mode
	public CANSparkMax getClawCanSparkMax() {
		return clawCanSparkMax;
	}

	//limits
	public boolean outerLimitReached() {
		return outerLeft.get() || outerRight.get();
	}
	
	public boolean innerLimitReached() {
		return innerRight.get() || innerLeft.get();
	}

	public void claw(double speed) {
		if ((outerLimitReached() && speed < 0) || (innerLimitReached() && speed > 0)) 
			{
			clawCanSparkMax.set(0);
			return;
		}
		clawCanSparkMax.set(speed);
    }

    public double getClawPosition() {
        return clawEncoder.getPosition();
    }

    public double getRotatePosition() {
        return rotateEncoder.getPosition();
    }

	public void rotate(double angle, double speed) {

        angle %= 360;
        
        if (angle < 0) {
            angle += 360;
        }

        if (rotateEncoder.getPosition() < angle) {
            rotateCANSparkMax.set(speed);
        } else {
            rotateCANSparkMax.set(-speed);
        }
	}
        
	public boolean getEncoderPos(double encoderLimit) {
		return clawEncoder.getPosition() == encoderLimit;
	}

	public boolean getRotate(double rotateLimit) {
		return rotateEncoder.getPosition() == rotateLimit;
	}

	public void resetEncoders() {
		clawEncoder.setPosition(0.0);
		// rotateEncoder.setPosition(0.0);
	}
}