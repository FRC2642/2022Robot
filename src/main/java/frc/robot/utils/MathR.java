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
    //https://www.desmos.com/calculator/vwfplwff43
    public static double proportion(double process, double deadband, double startSlowingAtRange, double stopWhenWithinRange, double maxOutput){
        double calculate = ((maxOutput-deadband)/(startSlowingAtRange-stopWhenWithinRange)) * (Math.abs(process)-stopWhenWithinRange) + deadband;

        if (calculate < deadband) return 0.0;
        else return Math.signum(process) * limit(calculate, 0.0, maxOutput);
    }
}
