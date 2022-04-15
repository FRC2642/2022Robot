// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MathR;

public class DriveDistanceCommand extends CommandBase {
  DriveSubsystem drive;
  double feet;
  double driveSpeed;
  double turnSpeed;

  PIDController turnPIDController = new PIDController(0.2, 0, 0);
  PIDController distancePIDController = new PIDController(0.2, 0, 0);

  public DriveDistanceCommand(DriveSubsystem drive, double feet, double driveSpeed, double turnSpeed) {
    this.drive = drive;
    this.feet = feet;
    this.driveSpeed = driveSpeed;
    this.turnSpeed = turnSpeed;
    distancePIDController.setSetpoint(feet);
    distancePIDController.setTolerance(0.1); //0.1 of a foot away from setpoint
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetEncoder();
    drive.resetPID();
    turnPIDController.setSetpoint(DriveSubsystem.getYaw());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   /* double rotationValue = drive.calculatePID(drive.getYaw(), setpoint);
    double speedValue = drive.calculatePID((drive.getEncoderDistance()/17117.0) * 18.8495559215, MathR.feetToInches(distance));
    if (rotationValue > 1) rotationValue = 1;
    if (rotationValue < -1) rotationValue = -1;
    drive.move(speedValue, rotationValue * 0.4);*/
    double speed = MathR.limit(distancePIDController.calculate(drive.getEncoderDistanceFeet()), -1.0, 1.0);
    double turn = MathR.limit(turnPIDController.calculate(DriveSubsystem.getYaw()), -1.0, 1.0);

    drive.move(speed * driveSpeed, turn * turnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end. 11027
  @Override
  public boolean isFinished() {
    return distancePIDController.atSetpoint();
  }
}