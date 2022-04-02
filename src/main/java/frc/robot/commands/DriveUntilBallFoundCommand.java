// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.drive.DriveSpeedCommand;
import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.commands.magazine.MagazineRunCommand;
import frc.robot.commands.waitfor.WaitForTwoBallsThere;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveUntilBallFoundCommand extends ParallelRaceGroup {

  public DriveUntilBallFoundCommand(DriveSubsystem drive, IntakeSubsystem intake, MagazineSubsystem mag, Command driver, Command waiter) {

    addCommands(
      driver,
      new RunIntakeCommand(intake),
      new MagazineRunCommand(mag, true), //true = run mag in reverse
      waiter
    );
  }
}
