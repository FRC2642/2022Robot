// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.fasterxml.jackson.databind.deser.DefaultDeserializationContext;

/** Add your docs here. */
public class MathR {
    public static double limit(double value, double floor, double ceiling){
        return Math.max(Math.min(ceiling, value), floor);
    }
    
    public static double feetToInches(double feet){
        return feet * 12;
    }
    //based on this https://www.desmos.com/calculator/qyqubv0lsu graph
    public static double ramp(double process, double deadband, double exp, double max, double offset){
        return 
            Math.signum(process) *
            Math.min(
                (1.0-deadband) * Math.pow((Math.abs(process) + offset),exp) + deadband
            , max);
    }
}
