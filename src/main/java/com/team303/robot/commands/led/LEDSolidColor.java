// // Copyright (c) 2022 Team 303

// package com.team303.robot.commands.led;

// import static com.team303.robot.Robot.ledStrip;

// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.CommandBase;

// public class LEDSolidColor extends CommandBase {
// 	Color color;

// 	public LEDSolidColor(Color color) {
// 		addRequirements(ledStrip);
// 		this.color = color;
// 	}

// 	@Override
// 	public void initialize() {
// 		// for every LED set a color
// 		for (var i = 0; i < ledStrip.ledBuffer.getLength(); i++) {
// 			ledStrip.ledBuffer.setLED(i, this.color);
// 		}
// 		// Send color data to LEDSubsytem
// 		ledStrip.writeData();
// 	}

// 	@Override
// 	public boolean runsWhenDisabled() {
// 		return true;
// 	}
// }