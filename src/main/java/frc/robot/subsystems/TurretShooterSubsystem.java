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
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  public double targetVelocity;


  /** Creates a new TurretShooterSubsystem. */
  //turret hood in here
  public TurretShooterSubsystem() {
    
    shooter = new CANSparkMax(Constants.TURRET_SHOOTER_ID, MotorType.kBrushless);
    this.encoder = shooter.getEncoder();
    pidController= shooter.getPIDController();
    kP = 1; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = 0;
    maxRPM = 5600;
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);
    
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    
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
       double p = SmartDashboard.getNumber("P Gain", 0);
       double i = SmartDashboard.getNumber("I Gain", 0);
       double d = SmartDashboard.getNumber("D Gain", 0);
       double iz = SmartDashboard.getNumber("I Zone", 0);
       double ff = SmartDashboard.getNumber("Feed Forward", 0);
       double max = SmartDashboard.getNumber("Max Output", 0);
       double min = SmartDashboard.getNumber("Min Output", 0);

       SmartDashboard.putNumber("shooter speed", getShooterSpeed());
   
       // if PID coefficients on SmartDashboard have changed, write new values to controller
       if((p != kP)) { pidController.setP(p); kP = p; }
       if((i != kI)) { pidController.setI(i); kI = i; }
       if((d != kD)) { pidController.setD(d); kD = d; }
       if((iz != kIz)) { pidController.setIZone(iz); kIz = iz; }
       if((ff != kFF)) { pidController.setFF(ff); kFF = ff; }
       if((max != kMaxOutput) || (min != kMinOutput)) { 
         pidController.setOutputRange(min, max); 
         kMinOutput = min; kMaxOutput = max; 
       }

       
    // This method will be called once per scheduler run
  }
}
