// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.VectorR;

public class VectorSubsystem extends SubsystemBase {
  
  double lastEncoderDistance = 0.0;
  final VectorR currentSample;
  final VectorR robotPosition;
  public double robotPlacedOnGroundDistanceFromHub = 5.0; //5 feet?
  /** Creates a new VectorSubsystem. */
  public VectorSubsystem() {
    currentSample = VectorR.fromCartesian(0.0, 0.0);
    robotPosition = VectorR.fromCartesian(robotPlacedOnGroundDistanceFromHub, 0.0);
    instance = this;
  }

  VectorSubsystem instance;
  @Override
  public void periodic() {
    currentSample.setFromPolar(lastEncoderDistance, Math.toRadians(DriveSubsystem.getYaw()));
    robotPosition.add(currentSample);

    lastEncoderDistance = DriveSubsystem.getEncoderDistanceFeet();

    SmartDashboard.putNumber("distance from start", getDistanceToHub());
    SmartDashboard.putNumber("angle to start", getAngleToHub());


    // This method will be called once per scheduler run
  }

  public double getAngleToHub(){
    return (Math.toDegrees(robotPosition.getAngle()) + 180) % 360;
  }
  
  public double getDistanceToHub(){
    return robotPosition.getMagnitude();
  }
}
