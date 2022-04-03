// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnByGyroAndPIDCommand extends CommandBase {
  /** Creates a new TurnDegreesCommand. */
  DriveSubsystem drive;
  double degrees;
  double setpoint;
  double rotationValue;
  public TurnByGyroAndPIDCommand(DriveSubsystem drive,  double degrees) {
    this.drive = drive;
    this.degrees = degrees;
    this.setpoint = degrees;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetPID();
    DriveSubsystem.resetYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotationValue = drive.calculatePID(DriveSubsystem.getYaw(), setpoint);
    if (rotationValue > 1){
      rotationValue = 1;
    }
    else if(rotationValue < -1){
      rotationValue = -1;
    }
    drive.move(0, rotationValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (degrees > 0){
      if (DriveSubsystem.getYaw() >= degrees){
        return true;
      }
      else{
        return false;
      }
    }
    else{
      if (DriveSubsystem.getYaw() <= degrees){
        return true;
      }
      else{
        return false;
      }
    }
  }
}