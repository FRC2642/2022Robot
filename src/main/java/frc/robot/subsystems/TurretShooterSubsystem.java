// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretShooterSubsystem extends SubsystemBase {

  private CANSparkMax shooter;

  /** Creates a new TurretShooterSubsystem. */
  //turret hood in here
  public TurretShooterSubsystem() {
    shooter = new CANSparkMax(Constants.TURRET_SHOOTER_ID, MotorType.kBrushless);
  }

  public void setSpeed(double speed){
    shooter.set(speed);
  }
  
  public void stop(){
    shooter.stopMotor();
  }

  public boolean getAuxRightTrigger() {
    double rtrigger = RobotContainer.auxController.getRightTriggerAxis();
    return (rtrigger > .5);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
