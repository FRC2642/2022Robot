// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class MagazineSubsystem extends SubsystemBase {
  /** Creates a new MagazineSubsystem. */
public CANSparkMax magBeltMotor;

  public MagazineSubsystem() {

    magBeltMotor = new CANSparkMax(10, MotorType.kBrushless);
  }

public void magRun(){
  magBeltMotor.set(0.6);
}
public void magStop(){
  magBeltMotor.set(0);
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}