// Copyright (c) 2022 Team 303

package com.team303.robot;

import com.team303.lib.math.DeadbandFilter;
import com.team303.swervelib.MechanicalConfiguration;

import edu.wpi.first.wpilibj.util.Color;

/*
TODO Change around all the CAN IDs to fit
*/

public final class RobotMap {

	public static final class Swerve {

		/* CAN IDs of Drive Motors */
		public static final int LEFT_FRONT_DRIVE_ID = 6;
		public static final int LEFT_BACK_DRIVE_ID = 3;
		public static final int RIGHT_FRONT_DRIVE_ID = 9;
		public static final int RIGHT_BACK_DRIVE_ID = 12;

		/* CAN IDs of steer Motors */
		public static final int LEFT_FRONT_STEER_ID = 4;
		public static final int LEFT_BACK_STEER_ID = 13;
		public static final int RIGHT_FRONT_STEER_ID = 7;
		public static final int RIGHT_BACK_STEER_ID = 10;

		/* Steer Encoder CAN IDs */
		public static final int LEFT_FRONT_STEER_CANCODER_ID = 5;
		public static final int LEFT_BACK_STEER_CANCODER_ID = 2;
		public static final int RIGHT_FRONT_STEER_CANCODER_ID = 8;
		public static final int RIGHT_BACK_STEER_CANCODER_ID = 11;

		/* Steer Motor Offset */
		public static final double LEFT_FRONT_STEER_OFFSET = Math.toRadians(-22.62);
		//public static final double LEFT_FRONT_STEER_OFFSET = Math.toRadians(-19.34);
		public static final double RIGHT_FRONT_STEER_OFFSET = Math.toRadians(-68.5);
		public static final double LEFT_BACK_STEER_OFFSET = Math.toRadians(-23.5);
		public static final double RIGHT_BACK_STEER_OFFSET = Math.toRadians(-208.3);

		/*Drive Train Dimentions*/
		public static final double TRACKWIDTH = 0.762;
		public static final double WHEELBASE = 0.762;
		public static final double ROTATION_CONSTANT = 2/Math.hypot(TRACKWIDTH, WHEELBASE);

		/* Motor Encoder Calculations */
		public static final double WHEEL_DIAMTER = 0.1524; // Diameter in meters
		// public static final int ENCODER_COUNTS_PER_REV = ; // ctre CANCoder
		public static final double DRIVE_GEAR_RATIO = 12.75; // Toughbox mini 12.75:1
		// public static final double DISTANCE_PER_ENCODER_PULSE; // Inches traveled for each encoder unit
		public static final double MAX_VELOCITY = 4;
			
		public static final double MAX_ACCELERATION = 3; //Meters per second
		public static final double MAX_RPS = 183.33; // Max rotations per second

		/* Starting Position */
		public static final double STARTING_X = 0;
		public static final double STARTING_Y = 0;

		/* Default Module Configurations */
		public static final MechanicalConfiguration MK4I_L2_LEFT_FRONT = new MechanicalConfiguration(
            0.10033,
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            true,
            (14.0 / 50.0) * (10.0 / 60.0),
            false
    	);

		public static final MechanicalConfiguration MK4I_L2_LEFT_BACK = new MechanicalConfiguration(
            0.10033,
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            false,
            (14.0 / 50.0) * (10.0 / 60.0),
            false
    	);

		public static final MechanicalConfiguration MK4I_L2_RIGHT_FRONT = new MechanicalConfiguration(
            0.10033,
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            true,
            (14.0 / 50.0) * (10.0 / 60.0),
            false
    	);

		public static final MechanicalConfiguration MK4I_L2_RIGHT_BACK = new MechanicalConfiguration(
            0.10033,
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            true,
            (14.0 / 50.0) * (10.0 / 60.0),
            false
    	);

		// static {
		// 	double wheelCircumference = WHEEL_DIAMTER * Math.PI;
		// 	double motorRotationsPerEncoderPulse = 1 / ENCODER_COUNTS_PER_REV;
		// 	double axelRotationsPerMotorRotation = 1 / MK4I_L2_LEFT_FRONT.getDriveReduction();

		// 	DISTANCE_PER_ENCODER_PULSE = motorRotationsPerEncoderPulse
		// 			* axelRotationsPerMotorRotation
		// 			* wheelCircumference;
		// }
	}

	public static final class Arm {

		public static final int SHOULDER_JOINT_RIGHT_ID = 14;
		public static final int SHOULDER_JOINT_LEFT_ID = 15;
		public static final int ELBOW_JOINT_ID = 16;
		public static final int CLAW_JOINT_ID = 17;
		public static final double SIMULATION_OFFSET = 0;
		public static final double SIMULATION_SCALE = 84/2.54;
		public static final double GEAR_RATIO_SHOULDER = 266.66;
		public static final double GEAR_RATIO_ELBOW = 125;
		public static final double GEAR_RATIO_CLAW = 45;
		// public static final double INCHES_TO_
		/*public static final int BOTTOM_LEFT_LIMIT_SWITCH = 0;
		public static final int BOTTOM_RIGHT_LIMIT_SWITCH = 1;
		public static final int TOP_RIGHT_LIMIT_SWITCH = 8;
		public static final int TOP_LEFT_LIMIT_SWITCH = 9;
		public static final int NEUTRAL_TOGGLE_SWITCH = 4;*/

		public static final float SOFT_LIMIT = 1000;

		public static final boolean CLIMB_MOTOR_INVERTED = true;
	}

	public static final class Claw {
		public static final int OUTER_LEFT_INPUT = 0;
		public static final int OUTER_RIGHT_INPUT = 1;
		public static final int INNER_LEFT_INPUT = 2;
		public static final int INNER_RIGHT_INPUT = 3;

		public static final int ULTRASONIC_ID = 1;

		public static final double ROTATE_OFFSET = Math.toRadians(0.0);
	}

	public static final class Auto {
		public static final double MAX_VELOCITY = 10;
		public static final double MAX_ACCELERATION = 10;
		public static final double RAMSETE_B = 10;
		public static final double RAMSETE_ZETA = 10;
	}
	
	public static final class PhotonvisionConstants {
		public static final double CAMERA_HEIGHT_METERS = 0; //NOT FINAL
		public static final double GRID_TARGET_HEIGHT_METERS = 0.36;
		public static final double DOUBLE_SUBSTATION_TARGET_HEIGHT_METERS = 0.59;
		public static final double CAMERA_PITCH_RADIANS = 0; //NOT FINAL
	}

	public static final class IOConstants {
		public static final int DRIVER_CONTROLLER = 0;
		public static final int OPERATOR_CONTROLLER = 1;

		public static final double DEADBAND_UPPERBOUND = 1;
		public static final double DEADBAND_LOWERBOUND = 0.15;
		public static final DeadbandFilter DEADBAND_FILTER = new DeadbandFilter(DEADBAND_LOWERBOUND, DEADBAND_UPPERBOUND);
	}

	public static final class LED {

		public static final int LED_ID = 1;
		public static final int BUFFER_LENGTH = 200;

		public static final Color RED = new Color(255 / 255D, 0, 0);
		public static final Color BLUE = new Color(0, 0, 255 / 255D);

		public static final Color FLASH_PRIMARY = new Color(
				255 / 255D,
				0 / 255D,
				0 / 255D);
		public static final Color FLASH_SECONDARY = new Color(
				230 / 255D,
				50 / 255D,
				0 / 255D);

		public static final Color TELEOP_COLOR = new Color(
				0 / 255D,
				255 / 255D,
				0 / 255D);
		public static final Color AUTONOMOUS_COLOR = new Color(
				230 / 255D,
				30 / 255D,
				0 / 255D);
		public static final Color DISABLED_COLOR = new Color(
				0 / 255D,
				90 / 255D,
				20 / 255D);
	}
}