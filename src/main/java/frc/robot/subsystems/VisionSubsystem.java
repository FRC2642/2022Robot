// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */

  //all vision (tape and ball) go in here (change variable names as necessary)
  public VisionSubsystem() {}

  public double getCenterX(){
    return Robot.centerX;
  }

  public double getCenterY(){
    return Robot.centerY;
  }

  public double getTargetArea(){
    return Robot.targetArea;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
