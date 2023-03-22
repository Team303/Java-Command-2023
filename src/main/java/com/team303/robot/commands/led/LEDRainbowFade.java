// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package com.team303.robot.commands.led;

// import static com.team303.robot.Robot.ledStrip;

// import edu.wpi.first.wpilibj2.command.CommandBase;


// public class LEDRainbowFade extends CommandBase {
// 	private int hue = 0;

// 	public LEDRainbowFade() {
// 		addRequirements(ledStrip);
// 	}

// 	@Override
// 	public void initialize() {
// 		ledStrip.clear();
// 	}

// 	@Override
// 	public void execute() {

// 		int hue = this.hue++ % 180;

// 		// for each singlar LED a assign a color
// 		for (var i = 0; i < ledStrip.ledBuffer.getLength(); i++) {
// 			ledStrip.ledBuffer.setHSV(i, hue, 255, 10);
// 		}

// 		// send the color to be used by the LEDSubsystem
// 		ledStrip.writeData();
// 	}

// 	@Override
// 	public void end(boolean interrupted) {
// 		ledStrip.clear();
// 	}

// 	@Override
// 	public boolean isFinished() {
// 		return false;
// 	}

// 	@Override
// 	public boolean runsWhenDisabled() {
// 		return true;
// 	}
// }