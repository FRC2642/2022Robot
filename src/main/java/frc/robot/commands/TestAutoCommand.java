// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TapeVisionSubsystem;
import frc.robot.subsystems.TurretShooterSubsystem;
import frc.robot.subsystems.TurretSpinnerSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAutoCommand extends SequentialCommandGroup {
  TurretShooterSubsystem turret;
  TurretSpinnerSubsystem turretSpinner;
  DriveSubsystem drive;
  TapeVisionSubsystem tapeVision;
  VisionSubsystem vision;
  public TestAutoCommand() {
    
    addCommands(new TurretSpinupCommand(turret));
    //Lower intake
    addCommands(new DriveDistanceCommand(drive, 8));
    addCommands(new TurnDistanceCommand(drive, 1));
    addCommands(new TurnTowardsHubCommand(turretSpinner, tapeVision));
    //Magazine
    //Shoot
    addCommands(new BallFollowerCommand(drive, vision));
    addCommands(new TurnTowardsHubCommand(turretSpinner, tapeVision));
    //Magazine
    //Shoot
  }
}
