// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VectorSubsystem extends SubsystemBase {

  public WPI_TalonFX backLeftMotor;
  public WPI_TalonFX backRightMotor;

  public Pigeon2 pigeon2;

  public double vectorComponentX = 0;
  public double vectorComponentY = 0; //consider making static
  public double lastEncoderPulses = 0;
  double magnitude = 0.0;
  double angle = 0.0;

  /** Creates a new VectorSubsystem. */
  public VectorSubsystem() {}
  public double getMagnitude(){
    //System.out.println("component x: " + (vectorComponentX));
    //System.out.println("component y: " + (vectorComponentY));
    //System.out.println("last encoder pulses: " + (lastEncoderPulses));
    magnitude = Math.sqrt(Math.pow(vectorComponentX - (8 * Constants.ENCODER_TICKS_PER_INCH), 2) + Math.pow(vectorComponentY, 2));
    return magnitude;
}

public double getAngle(){
    //System.out.println("Magnitude : " + (getMagnitude() / Constants.ENCODER_TICKS_PER_INCH));
    angle = (Math.acos((vectorComponentY / Constants.ENCODER_TICKS_PER_INCH) / (getMagnitude() / Constants.ENCODER_TICKS_PER_INCH)) * (180/Math.PI));
    return angle;
}

public double heading(){
  return pigeon2.getYaw();
}

public double getEncoderDistanceAverage(){
  double rightPulses = backRightMotor.getSelectedSensorPosition();
  double leftPulses = backLeftMotor.getSelectedSensorPosition();
  return (rightPulses + leftPulses) / 2.0;
}

public void reset(){
    vectorComponentY = 0;
    vectorComponentX = 0;
    lastEncoderPulses = 0;
}

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Vector Distance: ", magnitude);
    //SmartDashboard.putNumber("Vector Angle: ", angle);
    //SmartDashboard.putNumber("Vector Heading: ", heading());
    // This method will be called once per scheduler run
  }
}
