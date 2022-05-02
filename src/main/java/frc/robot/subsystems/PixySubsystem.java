// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PixySubsystem extends SubsystemBase {
  AnalogInput pixyAnalog = new AnalogInput(Constants.pixyCenterXPin);
	DigitalInput pixyDigital = new DigitalInput(Constants.pixyIsVisablePin);
  /** Creates a new PixySubsystem. */
  public PixySubsystem() {}

  public double getBallCenter() {
    return (pixyAnalog.getVoltage() / 3.3) * 315;
  }
  
  public boolean isBallVisible() {
    return pixyDigital.get();
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Center X: ", getBallCenter());
    //SmartDashboard.putBoolean("Is ball there: ", isBallVisible());
    // This method will be called once per scheduler run
  }
}
