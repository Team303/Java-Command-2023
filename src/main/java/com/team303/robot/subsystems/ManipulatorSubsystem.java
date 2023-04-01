package com.team303.robot.subsystems;

public interface ManipulatorSubsystem {

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
