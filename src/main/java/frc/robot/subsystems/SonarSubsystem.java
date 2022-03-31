// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SonarSubsystem extends SubsystemBase {
  /** Creates a new SonarSubsystem. */
  AnalogInput sonar = new AnalogInput(0);
  private static SonarSubsystem instance;
  //Ultrasonic sonar = new Ultrasonic(pingChannel, echoChannel);
  public SonarSubsystem() { instance = this; }

  //sensor readings
  public static double getSonarDistance(){
	  double valueToInches = 1 / 20.5;// 14.45
	  double distanceX = instance.sonar.getAverageValue();
	  double distance = (distanceX - 237) * valueToInches + 12; // convert voltage into inches
	  int distanceInt=(int) distance;
	  return distanceInt;

  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("sonar distance", getSonarDistance());

  }
}
