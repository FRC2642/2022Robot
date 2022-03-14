// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnDegreesCommand extends CommandBase {
  /** Creates a new TurnDegreesCommand. */
  DriveSubsystem drive;
  double rotationSpeed;
  double degrees;
  public TurnDegreesCommand(DriveSubsystem drive, double rotationSpeed, double degrees) {
    this.drive = drive;
    this.rotationSpeed = rotationSpeed;
    this.degrees = degrees;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.move(0, rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (degrees > 0){
      if (drive.getYaw() >= degrees){
        return true;
      }
      else{
        return false;
      }
    }
    else{
      if (drive.getYaw() <= degrees){
        return true;
      }
      else{
        return false;
      }
    }
  }
}
