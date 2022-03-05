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

//drive constants
public static final int FRONT_LEFT_TALON_ID = 1;
public static final int BACK_LEFT_TALON_ID = 2;
public static final int FRONT_RIGHT_TALON_ID = 3;
public static final int BACK_RIGHT_TALON_ID = 4;
public static final int TURRET_SHOOTER_ID = 5;
public static final int TURRET_SPINNER_ID = 6;

//throw math
public static final double FINAL_SPEED_MULT = 1;
public static final double TURRET_HEIGHT = 4; //meters
public static final double TARGET_HEIGHT = 8; //meters


//vision constants
public static final int MIN_NUM_PIXELS_RECT_SIMILARITY = 5;
public static final double[] HSL_HUE_RED = {0.0, 29.09090909090908};
public static final double[] HSL_SAT_RED = {105.48561151079136, 255.0};
public static final double[] HSL_LUM_RED = {0.0, 255.0};

public static final double[] HSL_HUE_BLUE = {91.0, 125.0};
public static final double[] HSL_SAT_BLUE = {80.0, 216.0};
public static final double[] HSL_LUM_BLUE = {0.0, 255.0};


//turret spinner constants
public static final int CLOCKWISE_SWITCH_ID = 8;
public static final int COUNTER_CLOCKWISE_SWITCH_ID = 9;
public static final int TURRET_SPIINER_MOTOR = 10;



}
