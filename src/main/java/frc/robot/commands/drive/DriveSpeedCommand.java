// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MathR;

public class DriveSpeedCommand extends CommandBase {
  DriveSubsystem drive;
  double speed;
  double startingAngle;
  
  public DriveSpeedCommand(DriveSubsystem drive, double speed, double angle) {
    this.drive = drive;
    this.speed = speed;
    startingAngle = angle;
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
    double turn =  MathR.proportion(-DriveSubsystem.getYaw(), 0.3, 180, 5, Math.abs(speed));
    drive.move(speed,0.0);
    SmartDashboard.putNumber("turn", turn);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end. 11027
  @Override
  public boolean isFinished() {
    return false;
  }
}