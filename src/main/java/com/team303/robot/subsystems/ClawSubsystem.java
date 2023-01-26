package com.team303.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.team303.robot.util.GroundedDigitalInput;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
	/* ShuffleBoard */
	public static final ShuffleboardTab CLIMBER_TAB = Shuffleboard.getTab("Climber");

	private final CANSparkMax clawCanSparkMax = new CANSparkMax(19, MotorType.kBrushless);
	private final RelativeEncoder clawEncoder;
	public final GroundedDigitalInput outerLeft;
	public final GroundedDigitalInput outerRight;
	public final GroundedDigitalInput innerLeft;
	public final GroundedDigitalInput innerRight;

	// private final int encoderGo;

	public ClawSubsystem(int encoderRotations) {
		clawCanSparkMax.setInverted(true);
		clawCanSparkMax.setIdleMode(IdleMode.kBrake);
		clawEncoder = clawCanSparkMax.getEncoder();

		// limit switches
		outerLeft = new GroundedDigitalInput(6);
		outerRight = new GroundedDigitalInput(9);
		innerLeft = new GroundedDigitalInput(15);
		innerRight = new GroundedDigitalInput(22);

		// encoderRotations = encoder

	}

	// claw motor getter among us imposter mode
	public CANSparkMax getClawCanSparkMax() {
		return clawCanSparkMax;
	}

	// limits
	public boolean outerLimitReached() {
		return outerLeft.get() || outerRight.get();
	}

	public boolean innerLimitReached() {
		return innerRight.get() || innerLeft.get();
	}

	public void close(double speed) {
		if ((outerLimitReached() && speed < 0) || (innerLimitReached() && speed > 0)) {
			// climbMotor.set(0);
			return;
		}
		// climbMotor.set(speed);
	}

	@Override
	public void periodic() {

	}
}
