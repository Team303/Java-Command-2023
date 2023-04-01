package com.team303.robot.subsystems;

import static com.team303.robot.autonomous.AutonomousProgram.AUTO_TAB;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase implements ManipulatorSubsystem {
    private static final ShuffleboardTab INTAKE_TAB = Shuffleboard.getTab("Intake");
    private static final GenericEntry intakeSwitchReverseEntry = INTAKE_TAB.add("INTAKE Switch Reverse", false)
            .getEntry();
    private static final GenericEntry stateEntry = AUTO_TAB.add("State", IntakeState.NONE.getName()).withPosition(0, 2)
            .getEntry();
    private static final GenericEntry modeEntry = AUTO_TAB.add("Mode", GamePieceType.CONE.getName()).withPosition(1, 2)
            .getEntry();
    public static final SendableChooser<IntakeState> intakeStateChooser = new SendableChooser<>();
    public static final SendableChooser<GamePieceType> intakeModeChooser = new SendableChooser<>();

    static {
        intakeStateChooser.setDefaultOption("None", IntakeState.NONE);
        intakeStateChooser.addOption("Intake", IntakeState.INTAKE);
        intakeStateChooser.addOption("Outtake", IntakeState.OUTTAKE);
        intakeModeChooser.addOption("Cube", GamePieceType.CUBE);
        intakeModeChooser.setDefaultOption("Cone", GamePieceType.CONE);
        AUTO_TAB.add("Open Close", intakeStateChooser).withPosition(0, 1);
        AUTO_TAB.add("Cone Cube", intakeModeChooser).withPosition(1, 1);
    }

    /* Gear Ratios */
    public static final double GEAR_RATIO_INTAKE = 50;

    /* Speed Constants */
    public static final double MAX_INTAKE_SPEED = 0.5;

    /* Motors */
    private final CANSparkMax intakeMotor = new CANSparkMax(19, MotorType.kBrushless);
    private final SparkMaxLimitSwitch intakeSwitchReverseLimit;

    public IntakeState state = IntakeState.NONE;

    public static enum IntakeState implements ManipulatorState {
        NONE,
        INTAKE,
        OUTTAKE;

        // state
        private String getName() {
            switch (this) {

                case INTAKE:
                    return "Intake";

                case OUTTAKE:
                    return "Outtake";
                default:
                    return "NONE";

            }
        }
    }

    public GamePieceType mode = GamePieceType.CONE;

    public IntakeSubsystem() {
        /* Claw Actuation Motor */
        // TODO: Check motor inversions
        intakeMotor.setInverted(true);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        //TODO: Set intake current limit
        // intakeMotor.setSmartCurrentLimit(20);

        intakeSwitchReverseLimit = intakeMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    }

    public void nextState() {
        switch (this.state) {
            case NONE:
                this.state = IntakeState.INTAKE;
                break;
            case INTAKE:
                this.state = IntakeState.OUTTAKE;
                break;
            case OUTTAKE:
                this.state = IntakeState.NONE;
        }
    }

    public void setState(ManipulatorState state) {
        this.state = (IntakeState) state;
    }

    public IntakeState getState() {
        return this.state;
    }

    public void toggleMode() {
        this.mode = this.mode == GamePieceType.CONE ? GamePieceType.CUBE : GamePieceType.CONE;
    }

    public void setMode(GamePieceType mode) {
        this.mode = mode;
    }

    public GamePieceType getMode() {
        return this.mode;
    }

    public void setManipulatorSpeed(double speed) {
        intakeMotor.set(speed * MAX_INTAKE_SPEED);
    }

    @Override
    public void periodic() {
        intakeSwitchReverseEntry.setBoolean(intakeSwitchReverseLimit.isPressed());
        stateEntry.setString(state.getName());
        modeEntry.setString(mode.getName());
    }

}
