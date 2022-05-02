// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class DriveStraightCommand extends DriveAtFixedHeadingCommand {

    public DriveStraightCommand(DriveSubsystem drive, double driveSpeed, double turnSpeed) {
        super(drive, driveSpeed, turnSpeed, 0.0);
    }

    @Override
    public void initialize() {
        angle = DriveSubsystem.getYaw();
    }

}
