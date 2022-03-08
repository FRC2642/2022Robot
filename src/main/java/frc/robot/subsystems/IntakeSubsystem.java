// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IntakeSubsystem extends SubsystemBase {

  CANSparkMax intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);
  CANSparkMax intakeBigwheel = new CANSparkMax(Constants.INTAKE_BIGWHEEL_ID, MotorType.kBrushless);
//unknown pneumatics module type - CTREPCM needs to be replaced
  DoubleSolenoid leftIntakePiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.INTAKE_FORWARDLEFTPISTON_ID, Constants.INTAKE_REVERSELEFTPISTON_ID);
  DoubleSolenoid rightIntakePiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.INTAKE_FORWARDRIGHTPISTON_ID, Constants.INTAKE_REVERSERIGHTPISTON_ID);

  /** Creates a new IntakeSubsystem. */
  //big wheel is going to go in intake

  public IntakeSubsystem() {

    //CHANGE THIS
    //intakePiston = new DoubleSolenoid(null, 0, 1);
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(false);
    //intakeMotor.setSmartCurrentLimit(kCurrentLimit);



  }

  
 /* public void extendIntake() {
    intakeMotor.set(Constants.INTAKE_WHEEL_SPEED);
    intakeBigwheel.set(Constants.INTAKE_WHEEL_SPEED);
    
    leftIntakePiston.set(Value.kForward);
    rightIntakePiston.set(Value.kForward);
  } */

  public void intakeMotorForward() {
    intakeMotor.set(Constants.INTAKE_WHEEL_SPEED);
  }

  public void intakeMotorReverse() {
    intakeMotor.set(-Constants.INTAKE_WHEEL_SPEED);
  }

  public void intakeBigwheelForward() {
    intakeBigwheel.set(Constants.BIG_WHEEL_SPEED);
  }

  public void intakeBigwheelReverse() {
    intakeBigwheel.set(-Constants.BIG_WHEEL_SPEED);
  }

  public void intakePistonExtend() {
    leftIntakePiston.set(Value.kForward);
    rightIntakePiston.set(Value.kForward); 
  }

  /*public void retractIntake() {
    intakeMotor.set(0);
    intakeBigwheel.set(0);

    leftIntakePiston.set(Value.kReverse);
    rightIntakePiston.set(Value.kReverse); 
  } */

  public void intakeMotorOff() {
    intakeMotor.set(0);
  }

  public void intakeBigwheelOff() {
    intakeBigwheel.set(0);
  }

  public void intakePistonRetract() {
    leftIntakePiston.set(Value.kReverse);
    rightIntakePiston.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
