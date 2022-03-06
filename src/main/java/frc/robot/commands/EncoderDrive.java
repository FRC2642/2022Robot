// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class EncoderDrive extends CommandBase {
  /** Creates a new EncoderDrive. */
  DriveSubsystem drive;
  double distance;

  public EncoderDrive(DriveSubsystem drive, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.distance = distance;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.move(0.6, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if(drive.getDriveEncoder() >= distance){
      return true;
    }
    else{
    return false;
    }
  }
}