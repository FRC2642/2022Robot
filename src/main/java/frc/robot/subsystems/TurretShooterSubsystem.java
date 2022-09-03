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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;


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
  private final int ledCount = 68;
  private final Animation rainbowAnimation = new RainbowAnimation(0.7, 0.8, ledCount);
  private final Animation blueAllianceLarsonAnimation = new LarsonAnimation(0, 0, 255, 0, 0.99, ledCount, BounceMode.Back, 7);
  private final Animation redAllianceLarsonAnimation = new LarsonAnimation(255, 0, 0, 0, 0.99, ledCount, BounceMode.Back, 7); 
 




  /** Creates a new TurretShooterSubsystem. */
  //turret hood in here
  public TurretShooterSubsystem() {
    instance = this;
    shooter = new CANSparkMax(Constants.TURRET_SHOOTER_ID, MotorType.kBrushless);
    this.encoder = shooter.getEncoder();
    pidController= shooter.getPIDController();
    kP = 0.003;//0.0037; 
    kI = 0.00000009;//2e-7; 
    kD = 1.2;//0.27; 
    kIz = 0.0;//0; 
    kFF = 0.00009;//0.000089285714;//0.5/5600; //0.000015; 
    kMaxOutput = 0.84;//0.8; 
    kMinOutput = 0.00;;//0;
    maxRPM = 5600;
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);
    //SmartDashboard.putNumber("P Gain", kP);
    //SmartDashboard.putNumber("I Gain", kI);
    //SmartDashboard.putNumber("D Gain", kD);
    //SmartDashboard.putNumber("I Zone", kIz);
    //SmartDashboard.putNumber("Feed Forward", kFF);
    //SmartDashboard.putNumber("Max Output", kMaxOutput);
    //SmartDashboard.putNumber("Min Output", kMinOutput);



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

  public void setRainbowAnimation(){
    candle.animate(rainbowAnimation);
  }
  
  @Override
  public void periodic() {
       // read PID coefficients from SmartDashboard
       //double p = SmartDashboard.getNumber("P Gain", 0);
       //double i = SmartDashboard.getNumber("I Gain", 0);
       //double d = SmartDashboard.getNumber("D Gain", 0);
       //double iz = SmartDashboard.getNumber("I Zone", 0);
       //double ff = SmartDashboard.getNumber("Feed Forward", 0);
       //double max = SmartDashboard.getNumber("Max Output", 0);
       //double min = SmartDashboard.getNumber("Min Output", 0);

       // if PID coefficients on SmartDashboard have changed, write new values to controller
       /*if((p != kP)) { pidController.setP(p); kP = p; }
       if((i != kI)) { pidController.setI(i); kI = i; }
       if((d != kD)) { pidController.setD(d); kD = d; }
       if((iz != kIz)) { pidController.setIZone(iz); kIz = iz; }
       if((ff != kFF)) { pidController.setFF(ff); kFF = ff; }
       if((max != kMaxOutput) || (min != kMinOutput)) { 
         pidController.setOutputRange(min, max); 
         kMinOutput = min; kMaxOutput = max; 
       }*/

       SmartDashboard.putNumber("shooter speed", getShooterSpeed());
       SmartDashboard.putBoolean("shooter ready", isCloseToSetRPM());

      /* if (isCloseToSetRPM()){
        candle.setLEDs(0, 255, 0); */
        candle.animate(rainbowAnimation);
        //RobotContainer.driveController.setRumble(RumbleType.kLeftRumble, 0.1);
     /*  }
       else if (DriverStation.getAlliance() == Alliance.Blue) {
        //candle.setLEDs(0, 0, 255);
        candle.animate(blueAllianceLarsonAnimation);
       }
       else{
         //candle.setLEDs(255, 0, 0);
         candle.animate(redAllianceLarsonAnimation);
       }
*/

       
        //pidController.setD(SmartDashboard.getNumber("d", kD));
       
    // This method will be called once per scheduler run
  }
}

