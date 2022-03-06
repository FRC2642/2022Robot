// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;


public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  //big wheel is going to go in intake
  public DoubleSolenoid intakePiston;
  public CANSparkMax intakeMotor;

  public IntakeSubsystem() {

    //CHANGE THIS
    //intakePiston = new DoubleSolenoid(null, 0, 1);
    intakeMotor = new CANSparkMax(5, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(false);
    //intakeMotor.setSmartCurrentLimit(kCurrentLimit);



    intakePiston = new DoubleSolenoid(null, 0, 1);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
