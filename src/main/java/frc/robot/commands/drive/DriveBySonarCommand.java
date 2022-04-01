// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SonarSubsystem;

public class DriveBySonarCommand extends CommandBase {
  private DriveSubsystem drive;
  private double distance;
  private double startingDistance;
  /** Creates a new DriveBySonarCommand. */
  public DriveBySonarCommand(DriveSubsystem drive, double distance) {
    this.drive = drive;
    this.distance = distance;
    this.startingDistance = SonarSubsystem.getSonarDistance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.move(Math.signum(SonarSubsystem.getSonarDistance() - distance)*0.375,0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(SonarSubsystem.getSonarDistance() - distance) < 5.0;
  }
}