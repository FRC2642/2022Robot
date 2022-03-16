// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.Constants;

/** Add your docs here. */
public class DistanceCalculator {
    private double inches;

    public double feetToInches(double feet){
        return feet * 12.0;
    }

}
