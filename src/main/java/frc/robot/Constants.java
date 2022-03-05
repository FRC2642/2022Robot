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

//intake constants
public static final int INTAKE_MOTOR_ID = 6;
public static final int INTAKE_BIGWHEEL_ID = 7;

public static final int INTAKE_FORWARDLEFTPISTON_ID = 8;
public static final int INTAKE_FORWARDRIGHTPISTON_ID = 9; 
public static final int INTAKE_REVERSELEFTPISTON_ID = 10;
public static final int INTAKE_REVERSERIGHTPISTON_ID = 11; 

public static final double INTAKE_WHEEL_SPEED = .1;


//vision constants
public static final int MIN_NUM_PIXELS_RECT_SIMILARITY = 5;



}
