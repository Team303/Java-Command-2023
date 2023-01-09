package frc.robot.autonomous;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AutonomousProgram {
	// A chooser for autonomous commands
	public static SendableChooser<AutonomousProgram> autoChooser = new SendableChooser<>();

	// HashMap registry to store all the created auto programs
	private static Map<String, AutonomousProgram> registry = new HashMap<>();

	// Most basic auto already defined
	public static AutonomousProgram DO_NOTHING = new AutonomousProgram("Do Nothing", () -> null);

	static {
		// Add basic auto to registry
		registry.put(DO_NOTHING.name, DO_NOTHING);
	}

	AutonomousProvider provider;
	String name;

	private AutonomousProgram(String name, AutonomousProvider provider) {
		this.provider = provider;
		this.name = name;
	}

	public String getName() {
		return this.name;
	}

	/**
	 * Creates an instance using the provider
	 * @return The instaniated base command of the auto (Might be null)
	 */
	public CommandBase construct() {
		return provider.construct();
	}

	/**
	 * Registers an auto program
	 * @param name The name to show in SmartDashboard
	 * @param provider A lambda which returns a command or null
	 */
	public static void create(String name, AutonomousProvider provider) {
		if (registry.get(name) != null)
			throw new IllegalArgumentException(
					String.format("Duplicate autonomous registered with name \"%s\"", name));

		registry.put(name, new AutonomousProgram(name, provider));
	}

	/**
	 * Basic Lambda for creating new command instances
	 * (so that autos that dont clean up arent broken)
	 * 
	 * @return
	 */
	interface AutonomousProvider {
		CommandBase construct();
	}

	/**
	 * Adds all values from registry to Shuffleboard
	 */
	public static void addAutosToShuffleboard() {
		// Add commands to the autonomous command chooser
		for (var auto : registry.values())
			autoChooser.addOption(auto.getName(), auto);

		System.out.println(registry);

		autoChooser.setDefaultOption(DO_NOTHING.getName(), DO_NOTHING);

		// Put the chooser on the dashboard
		Robot.AUTO_TAB.add("Autonomous Selector", autoChooser);
	}

}
