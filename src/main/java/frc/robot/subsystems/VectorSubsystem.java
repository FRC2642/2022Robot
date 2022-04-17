// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.VectorR;

public class VectorSubsystem extends SubsystemBase {
  
  final VectorR currentSample;
  final VectorR robotPosition;
  public double robotPlacedOnGroundDistanceFromHub = -7.0; //5 feet?
  private static VectorSubsystem instance;
  /** Creates a new VectorSubsystem. */
  public VectorSubsystem() {
    currentSample = VectorR.fromCartesian(0.0, 0.0);
    robotPosition = VectorR.fromCartesian(robotPlacedOnGroundDistanceFromHub, 0.0);
    instance = this;
  }

  
  double lastEncoderDistance = 0.0;
  @Override
  public void periodic() {
    currentSample.setFromPolar(lastEncoderDistance - DriveSubsystem.getEncoderDistanceFeet(), Math.toRadians(DriveSubsystem.getYaw() * -1));
    robotPosition.add(currentSample);

    lastEncoderDistance = DriveSubsystem.getEncoderDistanceFeet();

    SmartDashboard.putNumber("distance from start", getDistanceToHub());
    SmartDashboard.putNumber("angle to start", getAngleToHub());


    // This method will be called once per scheduler run
  }

  public static double getAngleToHub(){
    if (instance == null) return 0.0;
    return (Math.toDegrees(instance.robotPosition.getAngle()) + 180) % 360;
  }
  
  public static double getDistanceToHub(){
    if (instance == null) return 0.0;
    return instance.robotPosition.getMagnitude();
  }
  
}
