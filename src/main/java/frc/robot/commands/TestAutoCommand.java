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
import edu.wpi.first.wpilibj2.command.WaitCommand;

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
    


    //-----Get another ball and shoots them both at start-----\\

    //Start speeding up the shooter and drop intake
    addCommands(new ShooterCommand(turret).alongWith(new IntakeOutCommand(intake)));
    //Activate intake and drive 8 feet 
    addCommands(new IntakeSpinForwardCommand(intake).alongWith(new DriveDistanceCommand(drive, 8)));
    //Turn turret towards hub and turn left 90 degrees
    addCommands(new TurnDegreesCommand(drive, 0.5, -90).alongWith(new TurnTowardsHubCommand(turretSpinner, tapeVision))); 
    addCommands(new TurretHoodUpCommand(turretSpinner)); //Angles the hood
    addCommands(new BigWheelMove(intake)); //Bring ball into magazine
    addCommands(new MagOnCommand(magazine)); //Shoots
    //Turn big wheel and magazine off5
    addCommands(new BigWheelOffCommand(intake).alongWith(new MagOffCommand(magazine)));
    //Speed up turret and move ball into magazine
    addCommands(new ShooterCommand(turret));
    addCommands(new WaitCommand(2));
    addCommands(new MagazineRunCommand(magazine)); //Shoot
    
    //-----Second run for balls-----\\

    //Speed turret back up and look for balls
    addCommands(new ShooterCommand(turret).alongWith(new BallFollowerCommand(drive, vision)));
    //Drive and get ball while turret turns to hub
    addCommands(new DriveDistanceCommand(drive, 1).alongWith(new TurnTowardsHubCommand(turretSpinner, tapeVision))); 
    addCommands(new BigWheelMove(intake)); //Bring ball into magazine
    //Wait a litle longer for turret to spin up and turn big wheel off
    addCommands(new WaitCommand(1));
    addCommands(new BigWheelOffCommand(intake));
    addCommands(new MagOnCommand(magazine)); //Shoots
    addCommands(new MagOffCommand(magazine)); //Turns magazine off

    
  }
}
