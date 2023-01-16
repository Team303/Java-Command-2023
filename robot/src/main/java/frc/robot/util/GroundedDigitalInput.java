// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Extended DigitalInput class that inverts the output to work with grounded
 * limit switches
 */
public class GroundedDigitalInput extends DigitalInput {

	public GroundedDigitalInput(int channel) {
		super(channel);
	}

	@Override
	public boolean get() {
		return !super.get();
	}

}
