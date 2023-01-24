// Copyright (c) 2022 Team 303

package com.team303.robot.subsystems;

import com.team303.robot.RobotMap.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

	/**
	 * Serial communication port for sending data to the arduino over USB
	 */
	private final SerialPort arduino;

	/**
	 * Buffer for writing LED data
	 */
	public final AddressableLEDBuffer ledBuffer;

	private static LEDSubsystem instance = new LEDSubsystem();

	public LEDSubsystem() {
		// Create the buffer for writing pixel data
		this.ledBuffer = new AddressableLEDBuffer(LED.BUFFER_LENGTH);

		SerialPort arduino = null;

		// Assures that even if the connection cant be established, that the robot can
		// still function normally
		try {
			// Create the serial connection
			arduino = new SerialPort(9600, Port.kUSB1);

			// Only flush data when the buffer is full
			arduino.setWriteBufferMode(WriteBufferMode.kFlushWhenFull);
			// We only want to write this many bytes over the buffer
			arduino.setWriteBufferSize(LED.BUFFER_LENGTH * 3);

			// Send the new buffer to the arduino to set all LEDs to off
			clear();
		} catch (Exception ex) {
			ex.printStackTrace();
		}

		this.arduino = arduino;
	}

	public static LEDSubsystem getLED() {
		if (instance == null) {
			return new LEDSubsystem();
		}
		return instance;
	}

	/**
	 * Turns all the LEDs to off
	 */
	public void clear() {
		// for every LED set a color
		for (var i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setLED(i, new Color(0, 0, 0));
		}
		// Send color data to LEDSubsytem
		writeData();
	}

	/**
	 * Sends the current state of the buffer to the arduino
	 * 
	 * Does not clear the buffer after sending!
	 */
	public void writeData() {
		// If there is no arduino connected, just bail
		if (arduino == null)
			return;

		// Create a byte buffer that we will send over the wire
		byte[] buff = new byte[LED.BUFFER_LENGTH * 3];

		for (var i = 0; i < ledBuffer.getLength(); i++) {
			buff[i * 3 + 0] = (byte) ledBuffer.getLED(i).red;
			buff[i * 3 + 0] = (byte) ledBuffer.getLED(i).green;
			buff[i * 3 + 0] = (byte) ledBuffer.getLED(i).blue;
		}

		// Write the byte buffer and force the buffer to flush its contents
		arduino.write(buff, buff.length);
		arduino.flush();
	}
}
