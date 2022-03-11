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

//drive constants (correct can ids)
public static final int FRONT_LEFT_TALON_ID = 1;
public static final int BACK_LEFT_TALON_ID = 2;
public static final int FRONT_RIGHT_TALON_ID = 3;
public static final int BACK_RIGHT_TALON_ID = 4;

public static final int CLIMBER_ID = 14;

//turret shooter constants (correct can ids)
public static final int TURRET_SHOOTER_ID = 5;

//intake constants (correct can ids)
public static final int INTAKE_MOTOR_ID = 6;
public static final int INTAKE_BIGWHEEL_ID = 11;

//magazine motors (correct can id)
public static final int MAGAZINE_BELT_MOTOR_ID = 13;


    //incorrect piston ids
public static final int INTAKE_FORWARDLEFTPISTON_ID = 0;
//public static final int INTAKE_FORWARDRIGHTPISTON_ID = 0; 
public static final int INTAKE_REVERSELEFTPISTON_ID = 0;
//public static final int INTAKE_REVERSERIGHTPISTON_ID = 11; 

public static final double INTAKE_WHEEL_SPEED = .7;
public static final double BIG_WHEEL_SPEED = .7;

//turret spinner constants (correct can ids) (check limit switches)
public static final int TURRET_SPINNER_ID = 12;
public static final int CLOCKWISE_SWITCH_ID = 1;
public static final int COUNTER_CLOCKWISE_SWITCH_ID = 0;


//vision constants
public static final int VISION_OUTPUT_RES_X = 160;
public static final int VISION_OUTPUT_RES_Y = 120;

public static final int MIN_NUM_PIXELS_RECT_SIMILARITY = 15;
public static final double[] HSL_HUE_RED = {0.0, 29.09090909090908};
public static final double[] HSL_SAT_RED = {105.48561151079136, 255.0};
public static final double[] HSL_LUM_RED = {0.0, 255.0};

public static final double[] HSL_HUE_BLUE = {91.0, 125.0};
public static final double[] HSL_SAT_BLUE = {80.0, 216.0};
public static final double[] HSL_LUM_BLUE = {0.0, 255.0};

public static final double[] HSL_HUE_GREEN = {59.89208633093525, 80.17064846416382};
public static final double[] HSL_SAT_GREEN =  {121.53776978417265, 235.41808873720134};
public static final double[] HSL_LUM_GREEN = {158.22841726618705, 222.36348122866895};



public static final double[] HSL_HUE_WHITE = {0.0, 255.0};
public static final double[] HSL_SAT_WHITE = {0.0, 36.0};
public static final double[] HSL_LUM_WHITE = {217.0, 255.0};

//throw math
public static final double FINAL_SPEED_MULT = 1;
public static final double TURRET_HEIGHT = 4; //meters
public static final double TARGET_HEIGHT = 8; //meters


//gyro id
public static final int pigeonID = 18;

//button bindings
public static final int xButtonDrive = 3;
public static final int bButtonDrive = 2;

}
