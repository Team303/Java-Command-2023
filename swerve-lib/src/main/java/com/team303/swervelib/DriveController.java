package com.team303.swervelib;

public interface DriveController {
    Object getDriveMotor();

    void setReferenceVoltage(double voltage);

    double getStateVelocity();

    double getStateDistance();
}
