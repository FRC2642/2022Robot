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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.ColorFlowAnimation;


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

  //figure out can ids and led count
  private final CANdle candle = new CANdle(17, "rio");
  private final int ledCount = 300;
  private final Animation colorFlowAnimation = new ColorFlowAnimation(0, 255, 0, 0, 0.7, ledCount, Direction.Forward);




  /** Creates a new TurretShooterSubsystem. */
  //turret hood in here
  public TurretShooterSubsystem() {
    instance = this;
    shooter = new CANSparkMax(Constants.TURRET_SHOOTER_ID, MotorType.kBrushless);
    this.encoder = shooter.getEncoder();
    pidController= shooter.getPIDController();
    kP = 0.001; 
    kI = 2.6e-7; //8
    kD = 0.035;//0.035; 
    kIz = 0; 
    kFF = 0.000089285714;//0.5/5600; //0.000015; 
    kMaxOutput = 0.4; 
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

    //setReference()            this may be used to edit PID

    //configuration of leds
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.RGB;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(configAll, 100);

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

       if (isCloseToSetRPM()){
        candle.setLEDs(0, 255, 0);
        //candle.animate(colorFlowAnimation);
       }
       else if (DriverStation.getAlliance() == Alliance.Blue) {
        candle.setLEDs(0, 0, 255);
       }
       else{
         candle.setLEDs(255, 0, 0);
       }

       
        //pidController.setD(SmartDashboard.getNumber("d", kD));
       
    // This method will be called once per scheduler run
  }
}

