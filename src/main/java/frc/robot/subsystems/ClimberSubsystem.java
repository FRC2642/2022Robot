// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  CANSparkMax elevatorMotor = new CANSparkMax(0, MotorType.kBrushless);
  DoubleSolenoid climbPistonOne = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 1);
  DoubleSolenoid climbPistonTwo = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 1);
  Encoder elevatorEncoder = new Encoder(0, 0);
  public ClimberSubsystem() {}

  public void pistonFoward(){
    climbPistonOne.set(DoubleSolenoid.Value.kForward);
    climbPistonTwo.set(DoubleSolenoid.Value.kForward);
   }
   public void pistonBackward(){
     climbPistonOne.set(DoubleSolenoid.Value.kReverse);
     climbPistonTwo.set(DoubleSolenoid.Value.kReverse);
   }
   public void pistonOff(){
     climbPistonOne.set(DoubleSolenoid.Value.kOff);
     climbPistonTwo.set(DoubleSolenoid.Value.kOff);
   }
  public void moveElevator(double speed){
   elevatorMotor.set(speed);
  }
  public double getElevatorEncoder(){
    return elevatorEncoder.get();
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
