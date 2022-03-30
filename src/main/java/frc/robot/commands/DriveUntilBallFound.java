// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MagazineSubsystem;

public class DriveUntilBallFound extends CommandBase {
  DriveSubsystem drive;
  MagazineSubsystem mag;
  /** Creates a new DriveUntilBallFound. */
  public DriveUntilBallFound(DriveSubsystem drive, MagazineSubsystem mag) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.mag = mag;
    
    addRequirements(drive, mag);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.move(0.4, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (mag.areTwoBallsThere()){
      return true;
    }
    else{
    return false;
    }
  }
}
