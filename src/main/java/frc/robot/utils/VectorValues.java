// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.Constants;

/** Add your docs here. */

public class VectorValues {
    public static double vectorComponentX = 0;
    public static double vectorComponentY = 0; //consider making static
    public static double lastEncoderPulses = 0;


public static double getMagnitude(){
    System.out.println("component x: " + (vectorComponentX));
    System.out.println("component y: " + (vectorComponentY));
    System.out.println("last encoder pulses: " + (lastEncoderPulses));
    return Math.sqrt(Math.pow(vectorComponentX - (8 * Constants.ENCODER_TICKS_PER_INCH), 2) + Math.pow(vectorComponentY, 2));
}

public static double distanceInFeet(){
    return getMagnitude() / Constants.ENCODER_TICKS_PER_FOOT;
}


public static double getAngle(){
    System.out.println("Magnitude : " + (getMagnitude() / Constants.ENCODER_TICKS_PER_INCH));
    return (Math.acos((vectorComponentY / Constants.ENCODER_TICKS_PER_INCH) / (getMagnitude() / Constants.ENCODER_TICKS_PER_INCH)) * (180/Math.PI));
}

public static void reset(){
    vectorComponentY = 0;
    vectorComponentX = 0;
    lastEncoderPulses = 0;
}
/*

 VectorSubsystem vector;
  double encoderValue;
  double totalEncoderValue;
  double vectorComponentX;
  double vectorComponentY;
  double distance;

public void initialize() {
    vector.vectorComponentX += (Math.sin(180 * (Math.PI/180)) * Constants.ENCODER_TICKS_FOUR_FEET);
    vector.vectorComponentY += (Math.cos(180 * (Math.PI/180)) * Constants.ENCODER_TICKS_FOUR_FEET);
  }

  public void execute() {
    double currentPulses;
    currentPulses = DriveSubsystem.getVectorDistance();
    encoderValue = currentPulses - vector.lastEncoderPulses;
    vector.lastEncoderPulses = currentPulses;
    vector.vectorComponentX += (Math.sin(vector.heading() * (Math.PI/180)) * encoderValue);
    vector.vectorComponentY += (Math.cos(vector.heading() * (Math.PI/180)) * encoderValue);
  }

  */

}
