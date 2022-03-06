// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class MathR {
    public static double limit(double value, double floor, double ceiling){
        return Math.max(Math.min(ceiling, value), floor);
    }
    
}
