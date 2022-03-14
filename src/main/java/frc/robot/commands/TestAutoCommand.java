// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.TapeVisionSubsystem;
import frc.robot.subsystems.TurretShooterSubsystem;
import frc.robot.subsystems.TurretSpinnerSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.sql.Time;
import java.util.concurrent.TimeUnit;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAutoCommand extends SequentialCommandGroup {
  TurretShooterSubsystem turret;
  TurretSpinnerSubsystem turretSpinner;
  DriveSubsystem drive;
  TapeVisionSubsystem tapeVision;
  VisionSubsystem vision;
  IntakeSubsystem intake;
  MagazineSubsystem magazine;
  
  public TestAutoCommand() {
    
    //Get another ball and shoots them both at start
    addCommands(new ShooterCommand(turret)); //Start speeding up the shooter
    addCommands(new IntakeOutCommand(intake)); //Drop intake out
    addCommands(new IntakeSpinForwardCommand(intake)); //Activate intake
    addCommands(new DriveDistanceCommand(drive, 8)); //Drive 8 feet
    addCommands(new TurnDegreesCommand(drive, 0.5, -90)); //Turn towards hub some
    addCommands(new TurnTowardsHubCommand(turretSpinner, tapeVision)); //Turret turn towards hub
    addCommands(new TurretHoodUpCommand(turretSpinner)); //Angles the hood
    addCommands(new BigWheelMove(intake)); //Bring ball into magazine
    addCommands(new MagazineRunCommand(magazine)); //Shoots
    addCommands(new ShooterCommand(turret));
    addCommands(new BigWheelMove(intake));
    addCommands(new MagazineRunCommand(magazine));

    //Second run for balls
    addCommands(new ShooterCommand(turret)); //Speed turret back up
    addCommands(new BallFollowerCommand(drive, vision)); //Get more balls
    addCommands(new TurnTowardsHubCommand(turretSpinner, tapeVision)); //Turret face hub
    addCommands(new BigWheelMove(intake)); //Bring ball into magazine
    addCommands(new MagazineRunCommand(magazine)); //Shoots
    
  }
}
