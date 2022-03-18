// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class MagazineSubsystem extends SubsystemBase {
  /** Creates a new MagazineSubsystem. */
public CANSparkMax magBeltMotor;

DigitalInput lightSensor = new DigitalInput(0);

  public MagazineSubsystem() {

    magBeltMotor = new CANSparkMax(13, MotorType.kBrushless);
  }

public void magRun(){
  magBeltMotor.set(0.6);
}

public void magReverse(){
  magBeltMotor.set(-0.6);
}

public void magStop(){
  magBeltMotor.set(0);
}

public boolean getAuxLeftTrigger() {
  double ltrigger = RobotContainer.auxController.getLeftTriggerAxis();
  return (ltrigger > .5);
}

public boolean isBallThere(){
  return ! lightSensor.get();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("is ball there", isBallThere());
  }
}