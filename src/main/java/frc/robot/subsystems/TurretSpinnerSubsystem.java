// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TurretSpinnerSubsystem extends SubsystemBase {

  //creates limit switches for turret
  public DigitalInput clockwiseSwitch = new DigitalInput(2);
  public DigitalInput counterClockwiseSwitch = new DigitalInput(3 );//on when not pressed (inverted)
 // public CANSparkMax turretMotor = new CANSparkMax(Constants.TURRET_SPINNER_ID, MotorType.kBrushless);
  public Solenoid turretHood = new Solenoid(PneumaticsModuleType.REVPH, 15);


  /** Creates a new TurretSpinnerSubsystem. */
  public TurretSpinnerSubsystem() {}

  //incorporate vision for aiming

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void turnTurret(double speed) {
    //turretMotor.set(speed);
  }
  
  public boolean clockwiseSwitchOn() {
    return clockwiseSwitch.get();
  }

  public boolean counterClockwiseSwitchOn() {
    return counterClockwiseSwitch.get();
  }

  public void turretHoodUp(){
    turretHood.set(true);
  }
  public void turretHoodDown(){
    turretHood.set(false);
  }

  public void manuelTurnTurret(double speed) {
    if (speed > 0) {
      if (clockwiseSwitchOn()) {
        turnTurret(0);
      } else {
        turnTurret(speed);
      }
    } else {
      if (counterClockwiseSwitchOn()) {
        turnTurret(0);
      } else {
        turnTurret(speed);
      }
    }
  }
}
