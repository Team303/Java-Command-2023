package com.team303.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.team303.robot.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmTest extends SubsystemBase {

    CANSparkMax motor1;
    CANSparkMax motor2;

    public ArmTest() {
        motor1 = new CANSparkMax(15, MotorType.kBrushless);
        motor2 = new CANSparkMax(14, MotorType.kBrushless);

        motor1.setIdleMode(IdleMode.kCoast);
        motor2.setIdleMode(IdleMode.kCoast);
        
        motor1.setInverted(false);
        motor2.setInverted(true);
    }

    public void move(double speed) {
        motor1.set(speed);
        motor2.set(speed);
    }
}
