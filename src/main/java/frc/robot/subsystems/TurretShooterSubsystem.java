// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretShooterSubsystem extends SubsystemBase {

  private CANSparkMax shooter;
  private SparkMaxPIDController pidController;
  private RelativeEncoder encoder;
  public static boolean isCloseToSetRPM() {
    return Math.abs(instance.targetVelocity -instance.getShooterSpeed()) < instance.range;
  };
  private static TurretShooterSubsystem instance;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  public double targetVelocity;
  public double range = 75;

  /** Creates a new TurretShooterSubsystem. */
  //turret hood in here
  public TurretShooterSubsystem() {
    instance = this;
    shooter = new CANSparkMax(Constants.TURRET_SHOOTER_ID, MotorType.kBrushless);
    this.encoder = shooter.getEncoder();
    pidController= shooter.getPIDController();
    kP = 0.001; 
    kI = 5e-7; //8
    kD = 0.03; 
    kIz = 0; 
    kFF = 0.5/5600; //0.000015; 
    kMaxOutput = 1; 
    kMinOutput = 0;
    maxRPM = 5600;
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    SmartDashboard.putNumber("p", kP);
    SmartDashboard.putNumber("i", kI);
    SmartDashboard.putNumber("d", kD);
  }

  public void setSpeed(double speed){
    
    pidController.setReference(speed, CANSparkMax.ControlType.kVelocity);
    targetVelocity = speed;

  }

  public double getShooterSpeed(){
    return encoder.getVelocity();
  }
  
  public void stop(){
    shooter.stopMotor();
  }

  public boolean getAuxRightTrigger() {
    double rtrigger = RobotContainer.auxController.getRightTriggerAxis();
    return (rtrigger > .5);
  }
  
  @Override
  public void periodic() {
       // read PID coefficients from SmartDashboard

       SmartDashboard.putNumber("shooter speed", getShooterSpeed());
       SmartDashboard.putBoolean("shooter ready", isCloseToSetRPM());

       
       
        //pidController.setD(SmartDashboard.getNumber("d", kD));
       
    // This method will be called once per scheduler run
  }
}

