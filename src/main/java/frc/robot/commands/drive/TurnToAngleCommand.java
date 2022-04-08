// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class TurnToAngleCommand extends DriveAtFixedHeadingCommand {

    public TurnToAngleCommand(DriveSubsystem drive, double turnSpeed, double angle) {
        super(drive, 0.0, turnSpeed, angle);
    }

    @Override
  public boolean isFinished() {
    return Math.abs(DriveSubsystem.getYaw() - angle) < 5.0;
  }

}
