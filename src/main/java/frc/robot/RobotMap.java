// Copyright (c) 2022 Team 303

package frc.robot;

import edu.wpi.first.wpilibj.util.Color;

/*
TODO Change around all the CAN IDs to fit
*/

public final class RobotMap {

	public static final class Swerve {

		/* CAN IDs of Drive Motors */
		public static final int LEFT_FRONT_DRIVE_ID = 1;
		public static final int LEFT_BACK_DRIVE_ID = 2;
		public static final int RIGHT_FRONT_DRIVE_ID = 3;
		public static final int RIGHT_BACK_DRIVE_ID = 4;

		/*Drive Encoder CAN IDs*/
		public static final int LEFT_FRONT_DRIVE_CANCODER_ID = 13;
		public static final int LEFT_BACK_DRIVE_CANCODER_ID = 14;
		public static final int RIGHT_FRONT_DRIVE_CANCODER_ID = 15;
		public static final int RIGHT_BACK_DRIVE_CANCODER_ID = 16;

		/*CAN IDs of turn Motors*/
		public static final int LEFT_FRONT_STEER_ID = 5;
		public static final int LEFT_BACK_STEER_ID = 6;
		public static final int RIGHT_FRONT_STEER_ID = 7;
		public static final int RIGHT_BACK_STEER_ID = 8;

		/*Steer Encoder CAN IDs */
		public static final int LEFT_FRONT_STEER_CANCODER_ID = 9;
		public static final int LEFT_BACK_STEER_CANCODER_ID = 10;
		public static final int RIGHT_FRONT_STEER_CANCODER_ID = 11;
		public static final int RIGHT_BACK_STEER_CANCODER_ID = 12;

		/*Steer Motor Offset*/
		public static final double LEFT_FRONT_STEER_OFFSET= 0.0;
		public static final double RIGHT_FRONT_STEER_OFFSET = 0.0;
		public static final double LEFT_BACK_STEER_OFFSET = 0.0;
		public static final double RIGHT_BACK_STEER_OFFSET = 0.0;

		/* Drivebase Motor Inversion */
		public static final boolean LEFT_FRONT_SPARK_INVERTED = true;
		public static final boolean LEFT_BACK_SPARK_INVERTED = true;
		public static final boolean RIGHT_FRONT_SPARK_INVERTED = false;
		public static final boolean RIGHT_BACK_SPARK_INVERTED = false;

		/*Drive Train Dimentions*/
		public static final double TRACKWIDTH = 15;
		public static final double WHEELBASE = 15;
		public static final double ROTATION_CONSTANT = 2/Math.hypot(TRACKWIDTH, WHEELBASE);

		/* Motor Encoder Calculations */
		public static final double WHEEL_DIAMTER = 6; // Diameter in inches
		public static final int ENCODER_COUNTS_PER_REV = 4096; // ctre CANCoder
		public static final double DRIVE_GEAR_RATIO = 12.75; // Toughbox mini 12.75:1
		public static final double DISTANCE_PER_ENCODER_PULSE; // Inches traveled for each encoder unit

		/* Starting Position */
		public static final double STARTING_X = 0;
		public static final double STARTING_Y = 0;


		static {
			double wheelCircumference = WHEEL_DIAMTER * Math.PI;
			double motorRotationsPerEncoderPulse = 1 / ENCODER_COUNTS_PER_REV;
			double axelRotationsPerMotorRotation = 1 / DRIVE_GEAR_RATIO;

			DISTANCE_PER_ENCODER_PULSE = motorRotationsPerEncoderPulse
					* axelRotationsPerMotorRotation
					* wheelCircumference;
		}
	}

	public static final class Climber {

		public static final int CLIMB_PORT_ID = 7;
		public static final int BOTTOM_LEFT_LIMIT_SWITCH = 0;
		public static final int BOTTOM_RIGHT_LIMIT_SWITCH = 1;
		public static final int TOP_RIGHT_LIMIT_SWITCH = 8;
		public static final int TOP_LEFT_LIMIT_SWITCH = 9;
		public static final int NEUTRAL_TOGGLE_SWITCH = 4;

		public static final float SOFT_LIMIT = 1000;

		public static final boolean CLIMB_MOTOR_INVERTED = true;
	}

	public static final class IOConstants {

		public static final int LEFT_JOYSTICK_ID = 1;
		public static final int RIGHT_JOYSTICK_ID = 2;
		public static final int OPERATOR_CONTROLLER = 0;
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
