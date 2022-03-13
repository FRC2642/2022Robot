// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.Constants;

/** Add your docs here. */
public class BallThrowCalculator {
  //  public static final BallThrowCalculator Instance = new BallThrowCalculator();

    private double distance;

    public BallThrowCalculator(double distance){

    }

    private double getVelocityY(double time){
        return (Constants.TARGET_HEIGHT-Constants.TURRET_HEIGHT)/time + 4.9*time;
    }
    private double getVelocityX(double time){
        return distance/time; 
    }
    private Vector2d getVectorForAngle(double angle) {
        //some algebra here idk
        double timeParam = Math.sqrt((distance*(Math.tan(angle)-((Constants.TARGET_HEIGHT-Constants.TURRET_HEIGHT)/distance)))/4.9);
        
        return new Vector2d(getVelocityX(timeParam),getVelocityY(timeParam));
    }
    public VectorR calculate(double dist, double requestAngle) {
        distance = dist;
        return VectorR.fromVector2d(getVectorForAngle(requestAngle));
    }

}
