package com.team303.robot.modules;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Ultrasonic extends SubsystemBase  {
    AnalogInput ultrasonic;
    private final int MIN = 30;
    private final int MAX = 100;

    public Ultrasonic(int pin, int bits) {
        ultrasonic = new AnalogInput(pin);
        ultrasonic.setAverageBits(bits);
    }

    public int getDistanceCM() {
        int raw = ultrasonic.getAverageValue();
        return raw/255 * (MAX - MIN) + MIN;
    }

    public int getRawValue() {
        return ultrasonic.getAverageValue();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Ultrasonic", ultrasonic.getAverageValue());
    }
}
