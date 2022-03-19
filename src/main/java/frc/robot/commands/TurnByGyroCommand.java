// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnByGyroCommand extends CommandBase {
  DriveSubsystem drive;
  double targetHeading;
  double turnSpeed;
  boolean isTurningRight = false;
  /** Creates a new TurnByGyroCommand. */
  public TurnByGyroCommand(DriveSubsystem drive, double targetHeading, double turnSpeed) {
    this.drive = drive;
    this.turnSpeed = Math.abs(turnSpeed);
    this.targetHeading = targetHeading;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(targetHeading >= 0){
      drive.move(0.0, turnSpeed);
    }
    else if(targetHeading < 0){
      drive.move(0.0, turnSpeed * -1);
      isTurningRight = true;
    }
    else{
      drive.move(0.0, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isTurningRight && drive.getHeading() <= targetHeading){
    return true;
    }
    else if(!isTurningRight && drive.getHeading() >= targetHeading){
    return true;
    }
    else{
    return false;
    }
  }
}
