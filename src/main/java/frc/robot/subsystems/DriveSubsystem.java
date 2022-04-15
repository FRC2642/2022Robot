// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.utils.VectorValues;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import javax.management.RuntimeOperationsException;

//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import com.kauailabs.navx.frc.AHRS;



public class DriveSubsystem extends SubsystemBase {
  //Variables
  public double setpoint;

 // public AHRS navx = new AHRS(I2C.Port.kMXP);
  /** Creates a new DriveSubsystem. */
  
  //Objects
  WPI_TalonFX frontLeft = new WPI_TalonFX(Constants.FRONT_LEFT_TALON_ID); //.configVoltageCompSaturation(voltage, timeoutMs);
  public WPI_TalonFX backLeft = new WPI_TalonFX(Constants.BACK_LEFT_TALON_ID); //.configOpenloopRamp(seconds to full speed???)
  WPI_TalonFX frontRight = new WPI_TalonFX(Constants.FRONT_RIGHT_TALON_ID);
  public WPI_TalonFX backRight = new WPI_TalonFX(Constants.BACK_RIGHT_TALON_ID);

  
  MotorControllerGroup rightMotors = new MotorControllerGroup(frontRight, backRight);
  MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeft, backLeft);

  DifferentialDrive diffDrive = new DifferentialDrive(rightMotors, leftMotors);
  public PIDController PIDcontrol = new PIDController(0,0,0);

  public Pigeon2 pigeon2 = new Pigeon2(18);

  private static DriveSubsystem instance;
  //Constructor
  public DriveSubsystem() {
    instance = this;
    configDriveRamp(0.4);

    pigeon2.clearStickyFaults();
    pigeon2.setYaw(0.0);
    setpoint = 0;

    frontLeft.configFactoryDefault();
    backLeft.configFactoryDefault();
    frontRight.configFactoryDefault();
    backRight.configFactoryDefault();

    frontRight.setInverted(true);
    backRight.setInverted(true);


    
    


    frontLeft.setNeutralMode(NeutralMode.Brake);
    backLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);

    //frontLeft.setVoltage(3);


    /*frontLeft.configNeutralDeadband(0.01);
    backLeft.configNeutralDeadband(0.01);
    frontRight.configNeutralDeadband(0.01);
    backRight.configNeutralDeadband(0.01);*/


  }
  
  public void configDriveRamp(double ramp){
    frontLeft.configOpenloopRamp(ramp);
    backLeft.configOpenloopRamp(ramp);
    frontRight.configOpenloopRamp(ramp);
    backRight.configOpenloopRamp(ramp);
  }
  

  
  //Drive methods
  public void stop(){
    move(0, 0);
  }
  
 
  public void move(double speed, double rotation){
   
    diffDrive.arcadeDrive(speed, -rotation);

    //diffDrive.arcadeDrive(xSpeed, zRotation);
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

  public void resetPID(){
    PIDcontrol.reset();
  }
  
  
  
  
  //Encoder Methods
  public double getAverageEncoderDistance(){
    return (frontRight.getSelectedSensorPosition() + frontLeft.getSelectedSensorPosition()) / 2;
  }
  
  //Encoder Methods
  public double getEncoderDistanceFeet(){
    return getAverageEncoderDistance() / 11027.0;
  }

  public void resetEncoder(){
    

    frontRight.setSelectedSensorPosition(0);
    
    frontLeft.setSelectedSensorPosition(0);
    
    backRight.setSelectedSensorPosition(0);
    
    backLeft.setSelectedSensorPosition(0);
  }
  
  public static double getYaw(){
    return instance.pigeon2.getYaw() * -1;
  }
  public static void resetYaw(){
    instance.pigeon2.setYaw(0.0);
  }
  /*public static double getVectorDistance(){
    double leftDistance = instance.backLeft.getSelectedSensorPosition();
    double rightDistance = instance.backRight.getSelectedSensorPosition();
    return (leftDistance + rightDistance) / 2;
  }*/

  


  @Override
  public void periodic() {
    //double currentPulses = DriveSubsystem.getVectorDistance();
    //double encoderValue = currentPulses - VectorValues.lastEncoderPulses;
    /*VectorValues.lastEncoderPulses = currentPulses;
    VectorValues.vectorComponentX += (Math.sin(getYaw() * (Math.PI/180)) * encoderValue);
    VectorValues.vectorComponentY += (Math.cos(getYaw() * (Math.PI/180)) * encoderValue);
    SmartDashboard.putNumber("Distance in pulses", VectorValues.getMagnitude());
    SmartDashboard.putNumber("Feet away", VectorValues.distanceInFeet());
    SmartDashboard.putNumber("Angle:", VectorValues.getAngle());
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("encoder", getEncoderDistance());
    SmartDashboard.putNumber("gyro", getYaw());*/
  }
}
