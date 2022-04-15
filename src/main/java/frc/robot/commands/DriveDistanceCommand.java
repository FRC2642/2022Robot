// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MathR;

public class DriveDistanceCommand extends CommandBase {
  DriveSubsystem drive;
  double distance;
  double setpoint;
  public DriveDistanceCommand(DriveSubsystem drive, double distance, double setpoint) {
    this.drive = drive;
    this.distance = distance;
    this.setpoint = setpoint;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetEncoder();
    drive.resetPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationValue = drive.calculatePID(drive.getYaw(), setpoint);
    double speedValue = drive.calculatePID((drive.getEncoderDistance()/17117.0) * 18.8495559215, MathR.feetToInches(distance));
    if (rotationValue > 1) rotationValue = 1;
    if (rotationValue < -1) rotationValue = -1;
    if (speedValue > 1) speedValue = 1;
    if (speedValue < -1) speedValue = -1;
    drive.move(speedValue, rotationValue * 0.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end. 11027
  @Override
  public boolean isFinished() {
    if ((drive.getEncoderDistance()/17117.0) * 18.8495559215 < MathR.feetToInches(distance)){
      return false;
    }
    else{
      return true;
    }
  }
}