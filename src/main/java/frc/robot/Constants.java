// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	// Field constants
	public static final double SHOOTING_ZONE_INNER_RADIUS_METERS = 0.2;
	public static final double SHOOTING_ZONE_OUTER_RADIUS_METERS = 0.3;

	public static final double FIELD_VISION_TARGET_HEIGHT = 102;
	public static final double FIELD_UPPER_HUB_RADIUS = 20.0;

	// ROS Interface
	public static final String COPROCESSOR_ADDRESS = "10.0.88.35";
	public static final String COPROCESSOR_ADDRESS_SIMULATED = "127.0.0.1";
	public static final int COPROCESSOR_PORT = 5800;
	public static final double COPROCESSOR_TABLE_UPDATE_DELAY = 1.0 / 30.0;
	public static final double COPROCESSOR_PERIODIC_UPDATE_DELAY = 1.0 / 30.0;
	public static final double COPROCESSOR_PERIODIC_UPDATE_OFFSET = 0.01;
	public static final double COPROCESSOR_SLOW_PERIODIC_UPDATE_DELAY = 1.0 / 5.0;
	public static final double COPROCESSOR_SLOW_PERIODIC_UPDATE_OFFSET = 0.02;
}
