// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class BallVisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */

  //all vision (tape and ball) go in here (change variable names as necessary)


  public BallVisionSubsystem() {}

  private double centerX;
  private double centerY;

  public void setCenterX(double x){
    centerX = x;
  }
  public void setCenterY(double y){
    centerY = y;
  }

  public double getCenterX(){
    return centerX;
  }

  public double getCenterY(){
    return centerY;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
