// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
 

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  
  //check can ids
  TalonFX frontLeft = new TalonFX(Constants.FRONT_LEFT_TALON_ID);
  TalonFX backLeft = new TalonFX(Constants.BACK_LEFT_TALON_ID);
  TalonFX frontRight = new TalonFX(Constants.FRONT_RIGHT_TALON_ID);
  TalonFX backRight = new TalonFX(Constants.BACK_RIGHT_TALON_ID);


  public DriveSubsystem() {

    backLeft.set(TalonFXControlMode.Follower, frontLeft.getDeviceID());
    backRight.set(TalonFXControlMode.Follower, frontRight.getDeviceID());

  }
  

  public void setLeftSpeed(double speed) {
    frontLeft.set(TalonFXControlMode.PercentOutput, speed);
  }
  
  public void setRightSpeed(double speed){
    frontRight.set(TalonFXControlMode.PercentOutput, speed);
  }
  
  public void stop(){
    setLeftSpeed(0.0);
    setRightSpeed(0.0);
  }

  public void arcadeDrive(double speed, double turn) {
    //check negatives and positives (probably not right)
    turn = -turn;
    frontLeft.set(TalonFXControlMode.PercentOutput, turn, DemandType.ArbitraryFeedForward, speed);
    frontRight.set(TalonFXControlMode.PercentOutput, turn, DemandType.ArbitraryFeedForward, -speed);
  }
  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
