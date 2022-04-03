// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveWithPIDCommand extends CommandBase {
  DriveSubsystem drive;
  double setpoint;
  double speed;
  double rotationValue;
  public DriveWithPIDCommand(DriveSubsystem drive, double setpoint, double speed) {
    this.drive = drive;
    this.speed = speed;
    this.setpoint = setpoint;
  
    drive.setPIDCoefficients(0.2, 0, 0);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotationValue = drive.calculatePID(DriveSubsystem.getYaw(), setpoint);
    
    if (rotationValue > 1){
      rotationValue = 1;
    }
    else if (rotationValue < -1){
      rotationValue = -1;
    }

    drive.move(speed, rotationValue * 0.3);
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