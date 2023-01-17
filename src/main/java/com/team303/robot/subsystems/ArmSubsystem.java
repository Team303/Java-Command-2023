// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team303.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team303libs.kinematics.IKWrapper;


//import com.team303.lib.kinematics.IKWrapper;

public class ArmSubsystem extends SubsystemBase {

	public static final ShuffleboardTab CLIMBER_TAB = Shuffleboard.getTab("Climber");
	private static ArmSubsystem instance = new ArmSubsystem();
	private ArmSubsystem() {
		IKWrapper arm = new IKWrapper();
		arm.setArmLength(86.03f);
		arm.setSegmentLengthRatio(0,0.5f);
		arm.setSegmentLengthRatio(1,0.3125f);
		arm.setSegmentLengthRatio(2,0.1875f);
		arm.setSegmentLengths();
		arm.setAngleConstraint(0,45,45);
		arm.setAngleConstraint(1,135,135);
		arm.setAngleConstraint(2,135,135);
		arm.setSegmentInitialDirection(0,(float)Math.PI/2);
		arm.setSegmentInitialDirection(1,0f);
		arm.setSegmentInitialDirection(2,(float)-Math.PI/4);
		arm.initializeArm();
	}
	public static ArmSubsystem getArm() {
		return instance;
	}


	@Override
	public void periodic() {
	}
}
