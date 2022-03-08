// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
 

public class DriveSubsystem extends SubsystemBase {
  //Variables
  public double setpoint;
  /** Creates a new DriveSubsystem. */
  
  //Objects
  WPI_TalonFX frontLeft = new WPI_TalonFX(Constants.FRONT_LEFT_TALON_ID);
  WPI_TalonFX backLeft = new WPI_TalonFX(Constants.BACK_LEFT_TALON_ID);
  WPI_TalonFX frontRight = new WPI_TalonFX(Constants.FRONT_RIGHT_TALON_ID);
  WPI_TalonFX backRight = new WPI_TalonFX(Constants.BACK_RIGHT_TALON_ID);

  Encoder driveEncoder = new Encoder(1,2);


  //MotorControllerGroup fControllerGroup = new MotorControllerGroup(frontLeft, frontRight);

  
  MotorControllerGroup rightMotors = new MotorControllerGroup(frontRight, backRight);
  MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeft, backLeft);

  DifferentialDrive diffDrive = new DifferentialDrive(rightMotors, leftMotors);
  public PIDController PIDcontrol = new PIDController(0,0,0);

  public PigeonIMU pigeon = new PigeonIMU(Constants.pigeonID);


  //Constructor
  public DriveSubsystem() {
    setpoint = 0;

  }
  

  
  //Drive methods
  public void stop(){
    move(0, 0);
  }

  public void move(double speed, double rotation){
    diffDrive.arcadeDrive(speed, rotation);
  }
  

  //PID Methods
  public double calculatePID(double measurement, double setpoint){
    return PIDcontrol.calculate(measurement, setpoint);
  }
  
  public void setPIDCoefficients(double propCoefficient, double integralCoefficient, double deriativeCoefficient){
    PIDcontrol.setPID(propCoefficient, integralCoefficient, deriativeCoefficient);
  }

  public void setpointPID(double setpoint) {
    PIDcontrol.setSetpoint(setpoint);
  }
  
  //Gyro Methods
  public double getYaw(){
    return pigeon.getYaw();
  }

  public void resetGyro(){
    pigeon.setYaw(0.0);
  }

  public void resetEncoder(){
    driveEncoder.reset();
  }

  public double getDriveEncoder(){
    return driveEncoder.getDistance();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
