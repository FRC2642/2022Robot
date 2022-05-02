// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ResetEncoderCommand extends CommandBase {
  /** Creates a new ResetGyroCommand. */
  public ResetEncoderCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveSubsystem.resetEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
