// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MathR;

public class DriveDistanceCommand extends CommandBase {
  DriveSubsystem drive;
  double degrees;
  double distance;
  public DriveDistanceCommand(DriveSubsystem drive, double distance, double degrees) {
    this.drive = drive;
    this.distance = distance;
    this.degrees = degrees;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetEncoder();
    drive.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (drive.getYaw() < degrees - 2){
      drive.move(0.3, -0.3);
    }
    else if (drive.getYaw() > degrees + 2){
      drive.move(0.3, 0.3);
    }
    else{
      drive.move(0.3, 0);
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end. 11027
  @Override
  public boolean isFinished() {
    if ((drive.getEncoderDistance()/17117.0) * 18.8495559215 < MathR.feetToInches(5)){
      return false;
    }
    else{
      return true;
    }
  }
}
