package com.team303.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
	/* ShuffleBoard */
	private static final ShuffleboardTab CLAW_TAB = Shuffleboard.getTab("Claw");
	private static final GenericEntry clawPositionEntry = CLAW_TAB.add("Claw Position", 0).getEntry();
	private static final GenericEntry clawSwitchReverseEntry = CLAW_TAB.add("Claw Switch Reverse", false).getEntry();
	private static final GenericEntry stateEntry = CLAW_TAB.add("State", ClawState.OPEN.getName()).getEntry();
	private static final GenericEntry modeEntry = CLAW_TAB.add("Mode", GamePieceType.CONE.getName()).getEntry();

	/* Gear Ratios */
	public static final double GEAR_RATIO_CLAW = 50;

	/* Speed Constants */
	public static final double MAX_CLAW_SPEED = 0.25;

	/* Motors */
	private final CANSparkMax clawMotor = new CANSparkMax(19, MotorType.kBrushless);
	private final RelativeEncoder clawEncoder;
	private final SparkMaxLimitSwitch clawSwitchReverseLimit;

	/* State */

	public static enum ClawState {
		OPEN,
		ClOSED;

		private String getName() {
			return this == ClawState.OPEN ? "Open" : "Closed";
		}
	}

	private ClawState state = ClawState.OPEN;

	/* Mode */

	public static enum GamePieceType {
		CONE,
		CUBE;

		private String getName() {
			return this == GamePieceType.CONE ? "Cone" : "Cube";
		}
	}

	private GamePieceType mode = GamePieceType.CONE;

	public ClawSubsystem() {
		/* Claw Actuation Motor */

		clawMotor.setInverted(true);
		clawMotor.setIdleMode(IdleMode.kBrake);

		clawMotor.setSmartCurrentLimit(30);

		clawEncoder = clawMotor.getEncoder();
		clawEncoder.setPositionConversionFactor(1 / GEAR_RATIO_CLAW);

		clawSwitchReverseLimit = clawMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
	}

	/* Claw Open and Close State */

	public void toggleState() {
		this.state = this.state == ClawState.OPEN ? ClawState.ClOSED : ClawState.OPEN;
	}

	public void setState(ClawState state) {
		this.state = state;
	}

	public ClawState getState() {
		return this.state;
	}

	/* Claw Game Piece Mode */

	public void toggleMode() {
		this.mode = this.mode == GamePieceType.CONE ? GamePieceType.CUBE : GamePieceType.CONE;
	}

	public void setMode(GamePieceType mode) {
		this.mode = mode;
	}

	public GamePieceType getMode() {
		return this.mode;
	}

	/* Motors */

	public boolean outerLimitReached() {
		return clawSwitchReverseLimit.isPressed();
	}

	public double getClawPosition() {
		return clawEncoder.getPosition();
	}

	public void setClawSpeed(double speed) {
		clawMotor.set(speed * MAX_CLAW_SPEED);
	}

	public void setClawPosition(double position) {
		clawEncoder.setPosition(position);
	}

	@Override
	public void periodic() {
		clawPositionEntry.setDouble(clawEncoder.getPosition());
		clawSwitchReverseEntry.setBoolean(clawSwitchReverseLimit.isPressed());
		stateEntry.setString(state.getName());
		modeEntry.setString(mode.getName());
	}
}