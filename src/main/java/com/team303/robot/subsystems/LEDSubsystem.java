package com.team303.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static com.team303.robot.RobotMap.LED;
import static com.team303.robot.Robot.ledStrip;

public class LEDSubsystem extends SubsystemBase {

	/**
	 * Serial communication port for sending data to the arduino over USB
	 */

	/**
	 * Buffer for writing LED data
	 */
	public final AddressableLEDBuffer ledBuffer;

	public LEDSubsystem() {
		// Create the buffer for writing pixel data
		this.ledBuffer = new AddressableLEDBuffer(LED.BUFFER_LENGTH);

		AddressableLED rio = null;

		// Assures that even if the connection cant be established, that the robot can
		// still function normally
		try {
			// Create the serial connection
			rio = new AddressableLED(0);
			rio.setLength(ledBuffer.getLength());

			// Send the new buffer to the arduino to set all LEDs to off
			clear();
		} catch (Exception ex) {
			ex.printStackTrace();
		}

		rio.setData(ledBuffer);
		rio.start();
	}

	/**
	 * Turns all the LEDs to off
	 */
	public void clear() {
		// for every LED set a color
		for (var i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setLED(i, new Color(0, 0, 0));
		}
	}

}