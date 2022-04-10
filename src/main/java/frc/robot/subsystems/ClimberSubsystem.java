// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  CANSparkMax elevatorMotor = new CANSparkMax(14, MotorType.kBrushless);
  Solenoid climbPistonOne = new Solenoid(PneumaticsModuleType.REVPH, 8);

  Solenoid climberTiltPiston1 = new Solenoid(PneumaticsModuleType.REVPH, 14);
  Solenoid climberTiltPiston2 = new Solenoid(PneumaticsModuleType.REVPH, 13);

  //Encoder elevatorEncoder = new Encoder(0, 0);
  public ClimberSubsystem() {
    elevatorMotor.restoreFactoryDefaults();
  }

  public void climbPistonFoward(){
    climbPistonOne.set(true);
    //climbPistonTwo.set(DoubleSolenoid.Value.kForward);
   }
   public void climbPistonBackward(){
     climbPistonOne.set(false);
     //climbPistonTwo.set(DoubleSolenoid.Value.kReverse);
   }
   
   public void climbTiltPiston1Foward(){
    climberTiltPiston1.set(true);
   }
   public void climbTiltPiston1Backward(){
    climberTiltPiston1.set(false);
   }

   public void climbTiltPiston2Foward(){
    climberTiltPiston2.set(true);
   }
   public void climbTiltPiston2Backward(){
    climberTiltPiston2.set(false);
   }

  public void moveElevator(double speed){
   elevatorMotor.set(1.0);
  }

  public void moveElevatorDown(double downSpeed){
    elevatorMotor.set(-1.0);
  }

  public void climberStop(){
    elevatorMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
