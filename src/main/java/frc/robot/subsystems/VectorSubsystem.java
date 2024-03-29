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
  public static double robotPlacedOnGroundDistanceFromHub = 7.0; //feet
  public static boolean robotPlacedFacingTowardsHub = true;
  private static VectorSubsystem instance;
  /** Creates a new VectorSubsystem. */
  public VectorSubsystem() {
    currentSample = VectorR.fromCartesian(0.0, 0.0);
    robotPosition = VectorR.fromCartesian(robotPlacedFacingTowardsHub ? -robotPlacedOnGroundDistanceFromHub : robotPlacedOnGroundDistanceFromHub, 0.0);
    instance = this;
  }

  
  double lastEncoderDistance = 0.0;
  @Override
  public void periodic() {
    currentSample.setFromPolar(DriveSubsystem.getEncoderDistanceFeet() - lastEncoderDistance, Math.toRadians(DriveSubsystem.getYaw() * -1));
    robotPosition.add(currentSample);

    lastEncoderDistance = DriveSubsystem.getEncoderDistanceFeet();

    SmartDashboard.putNumber("Hub Distance", getDistanceToHub());
    SmartDashboard.putNumber("Hub Heading", getAngleToHub());


    // This method will be called once per scheduler run
  }

  public static double getAngleToHub(){
    if (instance == null) return 0.0;
    return (Math.toDegrees(instance.robotPosition.getAngle() * -1) + 180) % 360;
  }
  
  public static double getDistanceToHub(){
    if (instance == null) return 0.0;
    return instance.robotPosition.getMagnitude();
  }

  public static void reset(){
    if (instance == null) return;
    instance.robotPosition.setFromCartesian(robotPlacedFacingTowardsHub ? -robotPlacedOnGroundDistanceFromHub : robotPlacedOnGroundDistanceFromHub, 0.0);
  }
  
}
