package com.team303.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team303.robot.autonomous.AutonomousProgram.AUTO_TAB;

public class ClawSubsystem extends SubsystemBase implements ManipulatorSubsystem {
	/* ShuffleBoard */
	private static final ShuffleboardTab CLAW_TAB = Shuffleboard.getTab("Claw");
	private static final GenericEntry clawPositionEntry = CLAW_TAB.add("Claw Position", 0).getEntry();
	private static final GenericEntry clawSwitchReverseEntry = CLAW_TAB.add("Claw Switch Reverse", false).getEntry();
	private static final GenericEntry stateEntry = AUTO_TAB.add("State", ClawState.OPEN.getName()).withPosition(0, 2)
			.getEntry();
	private static final GenericEntry modeEntry = AUTO_TAB.add("Mode", GamePieceType.CONE.getName()).withPosition(1, 2)
			.getEntry();
	public static final SendableChooser<ClawState> clawStateChooser = new SendableChooser<>();
	public static final SendableChooser<GamePieceType> clawModeChooser = new SendableChooser<>();

	static {
		clawStateChooser.setDefaultOption("Open", ClawState.OPEN);
		clawStateChooser.addOption("Closed", ClawState.ClOSED);
		clawModeChooser.addOption("Cube", GamePieceType.CUBE);
		clawModeChooser.setDefaultOption("Cone", GamePieceType.CONE);
		AUTO_TAB.add("Open Close", clawStateChooser).withPosition(0, 1);
		AUTO_TAB.add("Cone Cube", clawModeChooser).withPosition(1, 1);
	}

	/* Gear Ratios */
	public static final double GEAR_RATIO_CLAW = 50;

	/* Speed Constants */
	public static final double MAX_CLAW_SPEED = 0.5;

	/* Motors */
	private final CANSparkMax clawMotor = new CANSparkMax(19, MotorType.kBrushless);
	private final RelativeEncoder clawEncoder;
	private final SparkMaxLimitSwitch clawSwitchReverseLimit;

	/* State */
	public ClawState state = ClawState.OPEN;

	public static enum ClawState implements ManipulatorState {
		OPEN,
		ClOSED;

		// state
		private String getName() {
			return this == ClawState.OPEN ? "Open" : "Closed";
		}
	}

	public GamePieceType mode = GamePieceType.CONE;

	public ClawSubsystem() {
		/* Claw Actuation Motor */

		clawMotor.setInverted(false);
		clawMotor.setIdleMode(IdleMode.kBrake);

		clawMotor.setSmartCurrentLimit(20);

		clawEncoder = clawMotor.getEncoder();
		clawEncoder.setPositionConversionFactor(1 / GEAR_RATIO_CLAW);

		clawSwitchReverseLimit = clawMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
	}

	/* Claw Open and Close State */

	public void nextState() {
		this.state = this.state == ClawState.OPEN ? ClawState.ClOSED : ClawState.OPEN;
	}

	public void setState(ManipulatorState state) {
		this.state = (ClawState) state;
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

	public void setManipulatorSpeed(double speed) {
		clawMotor.set(speed * MAX_CLAW_SPEED);
	}

	public void setManipulatorPosition(double position) {
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