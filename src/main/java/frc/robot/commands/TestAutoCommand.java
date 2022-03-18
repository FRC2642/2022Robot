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
    addCommands(new ShooterCommand(turret).alongWith(new IntakeOutCommand(intake)),
    new IntakeSpinForwardCommand(intake).alongWith(new DriveDistanceCommand(drive, 8)),
    new TurnDegreesCommand(drive, 0.5, -180),
    new TurretHoodUpCommand(turretSpinner),
    new BigWheelMove(intake),
    new MagOnCommand(magazine),
    new BigWheelOffCommand(intake).alongWith(new MagOffCommand(magazine)),
    new ShooterCommand(turret),
    new WaitCommand(2),
    new MagazineRunCommand(magazine),

    new ShooterCommand(turret).alongWith(new BallFollowerCommand(drive, vision)),
    new DriveDistanceCommand(drive, 1),
    new WaitCommand(1),
    new BigWheelMove(intake),
    new BigWheelOffCommand(intake),
    new MagOnCommand(magazine),
    new MagOffCommand(magazine)
    );
  }
}














