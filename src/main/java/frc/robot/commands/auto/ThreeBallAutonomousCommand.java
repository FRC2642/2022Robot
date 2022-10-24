// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BallFollowerCommand;
import frc.robot.commands.ResetEncoderCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.ResetVectorCommand;
import frc.robot.commands.TimedShootCommand;
import frc.robot.commands.TurretHoodUpCommand;
import frc.robot.commands.drive.DriveAtFixedHeadingCommand;
import frc.robot.commands.drive.DriveBySonarCommand;
import frc.robot.commands.drive.DriveDistanceCommand;
import frc.robot.commands.drive.DriveSpeedCommand;
import frc.robot.commands.drive.DriveStraightCommand;
import frc.robot.commands.drive.TurnToAngleCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.intake.IntakePistonExtendCommand;
import frc.robot.commands.intake.IntakePistonRetractCommand;
import frc.robot.commands.intake.IntakeSpinForwardCommand;
import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.commands.magazine.MagazineRunCommand;
import frc.robot.commands.magazine.TimedMagazineRunCommand;
import frc.robot.commands.shooter.StartShooterCommand;
import frc.robot.commands.waitfor.WaitForOneBallThere;
import frc.robot.commands.waitfor.WaitForOneBallThere;
import frc.robot.commands.waitfor.WaitForRPMReachedCommand;
import frc.robot.commands.waitfor.WaitForTwoBallsThere;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.TurretShooterSubsystem;
import frc.robot.subsystems.TurretSpinnerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallAutonomousCommand extends SequentialCommandGroup {
  /** Creates a new AutonomousCommandGroup. */
  public ThreeBallAutonomousCommand(TurretShooterSubsystem turretShooter, IntakeSubsystem intake, DriveSubsystem drive, MagazineSubsystem mag, TurretSpinnerSubsystem spinner) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetEncoderCommand(),
      new ResetGyroCommand(),
      //new ResetVectorCommand(7.0,true),
      new TurretHoodUpCommand(spinner),
      new StartShooterCommand(turretShooter, 1100),
      new WaitForRPMReachedCommand(),
      new TimedShootCommand(mag, intake, 0.75),
      new StartShooterCommand(turretShooter, 0.0),
      new TurnToAngleCommand(drive, 0.4, 190),
      new BallFollowerCommand(drive).withTimeout(1.0),
      new DriveDistanceCommand(drive, 2.7, 0.45, 0.3),
      new IntakePistonExtendCommand(intake),
      new DriveSpeedCommand(drive, 0.3, 0.0).alongWith(new RunIntakeCommand(intake)).until(MagazineSubsystem::isOneBallThere).withTimeout(3.0),
      new IntakePistonRetractCommand(intake),
      new TurnToAngleCommand(drive, 0.4, 103).alongWith(new TimedShootCommand(mag, intake, 1)),
      new BallFollowerCommand(drive).withTimeout(1.0),
<<<<<<< HEAD
      new DriveDistanceCommand(drive, 4.75, 0.45, 0.3),
=======
      new DriveDistanceCommand(drive, 4.75, 0.5, 0.3),
>>>>>>> 709bfd80eb4cbff0191bfc9023af28ac053bd7d4
      new IntakePistonExtendCommand(intake),
      new StartShooterCommand(turretShooter, 1250),
      new DriveSpeedCommand(drive, 0.3, 0.0).alongWith(new RunIntakeCommand(intake), new MagazineRunCommand(mag, true)).until(MagazineSubsystem::areTwoBallsThere).withTimeout(3.0),
      new IntakePistonRetractCommand(intake),
      new TurnToAngleCommand(drive, 0.45, -15.5),
      new TurretHoodUpCommand(spinner),
      new WaitForRPMReachedCommand(),
      new TimedShootCommand(mag, intake, 3.5)
       );

  }
}
