package com.team303.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ManipulatorSubsystem extends Subsystem {

    enum GamePieceType {
        CONE,
        CUBE;

        public String getName() {
            return this == GamePieceType.CONE ? "Cone" : "Cube";
        }
    }

    interface ManipulatorState {
    }

    void setState(ManipulatorState state);

    ManipulatorState getState();

    void nextState();

    void toggleMode();

    void setMode(GamePieceType mode);

    GamePieceType getMode();

    void setManipulatorSpeed(double speed);

}
