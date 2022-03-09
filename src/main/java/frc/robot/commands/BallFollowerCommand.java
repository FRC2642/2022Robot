// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.BallVisionSubsystem;

public class BallFollowerCommand extends CommandBase {
  DriveSubsystem drive;
  BallVisionSubsystem vision;
  double error;
  /** Creates a new BallFollowerCommand. */
  public BallFollowerCommand(DriveSubsystem drive, BallVisionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.vision = vision;

    addRequirements(drive,vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    double speed = 0.25;
    if(vision.getCenterX() < 70)  drive.arcadeDrive(0, -speed); 
    else if(vision.getCenterX() > 90)  drive.arcadeDrive(0, speed); 
    else drive.arcadeDrive(0, 0);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
