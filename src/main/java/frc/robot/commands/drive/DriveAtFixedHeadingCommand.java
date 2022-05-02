// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MathR;

public class DriveAtFixedHeadingCommand extends CommandBase {
  DriveSubsystem drive;
  double angle;
  double driveSpeed;
  double turnPIDSpeed;
  public DriveAtFixedHeadingCommand(DriveSubsystem drive, double driveSpeed, double turnPIDSpeed, double angle) {
    this.drive = drive;
    this.driveSpeed = driveSpeed;
    this.turnPIDSpeed = turnPIDSpeed;
    this.angle = angle;
  
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    drive.setPIDCoefficients(0.075, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turn = MathR.limit(drive.calculatePID(DriveSubsystem.getYaw(), angle), -1.0, 1.0);
    
    drive.move(driveSpeed, turn * turnPIDSpeed);
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}