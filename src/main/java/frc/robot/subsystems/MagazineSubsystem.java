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

DigitalInput upperLightSensor = new DigitalInput(0);
DigitalInput lowerLightSensor = new DigitalInput(1);

private static MagazineSubsystem instance;

  public MagazineSubsystem() {
    instance = this;
    magBeltMotor = new CANSparkMax(13, MotorType.kBrushless);
  }

public void magRun(){
  magBeltMotor.set(0.9);
}

public void magReverse(){
  magBeltMotor.set(-1.0);
}

public void magStop(){
  magBeltMotor.set(0);
}

public boolean getAuxLeftTrigger() {
  double ltrigger = RobotContainer.auxController.getLeftTriggerAxis();
  return (ltrigger > .5);
}

//sensor methods
public static boolean isOneBallThere(){
  if (instance == null) return false;
  return  instance.upperLightSensor.get();
}

public static boolean areTwoBallsThere(){
  if (instance == null) return false;
  return  instance.lowerLightSensor.get() && instance.upperLightSensor.get();
}

public static boolean getLowerLightSensor(){
  if (instance == null) return false;
  return  instance.lowerLightSensor.get();
}
public static boolean getUpperLightSensor(){
  if (instance == null) return false;
  return  instance.upperLightSensor.get();
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("upper light sensor", getUpperLightSensor());
    SmartDashboard.putBoolean("lower light sensor", getLowerLightSensor());

  }
}