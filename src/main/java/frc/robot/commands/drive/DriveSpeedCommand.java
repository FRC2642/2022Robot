// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

//Drives at a specific speed/turn
public class DriveSpeedCommand extends CommandBase {
  DriveSubsystem drive;
  double speed;
  double turn;
  /** Creates a new DriveSpeedCommand. */
  public DriveSpeedCommand(DriveSubsystem drive, double speed, double turn) {
    this.drive = drive;
    this.speed = speed;
    this.turn = turn;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.move(speed, turn);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
