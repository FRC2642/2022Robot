// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TapeVisionSubsystem;
import frc.robot.subsystems.TurretShooterSubsystem;
import frc.robot.subsystems.TurretSpinnerSubsystem;

public class AutoTurretSpeedCommand extends CommandBase {
  TurretShooterSubsystem turret;
  TapeVisionSubsystem tapeVision;
  TurretSpinnerSubsystem spinner;
  double speedValue;
  double max = 4500;
  double min = 1000;
  public static Object[] tape;
  public AutoTurretSpeedCommand(TurretShooterSubsystem turret, TapeVisionSubsystem tapeVision, TurretSpinnerSubsystem spinner) {
    this.turret = turret;
    this.spinner = spinner;
    this.tapeVision = tapeVision;
    addRequirements(turret, spinner, tapeVision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (tapeVision.hubInFrame(new Vector2d(), new Vector2d()) == TapeVisionSubsystem.HubInFrameReason.DETECTED){
      tape = TapeVisionSubsystem.object;
      
      
      System.out.println(tape.length);
      if (tape.length >= 10){//Replace with realistic short range length
        speedValue = max / (tape.length * 0.25);
        spinner.turretHoodDown();
      }
      else{
        speedValue = max / (tape.length * 0.2);
        spinner.turretHoodUp();
      }
      //speedValue = max / (tape.length * 0.3); //Could be a good formula
      if (speedValue > max){
        speedValue = max;
      }
      else if (speedValue < min){
        speedValue = min;
      }
      turret.setSpeed(speedValue);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
