// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveBySonarCommand;
import frc.robot.commands.drive.DriveSpeedCommand;
import frc.robot.commands.drive.DriveStraightCommand;
import frc.robot.commands.drive.TurnToAngleCommand;
import frc.robot.commands.intake.IntakePistonExtendCommand;
import frc.robot.commands.intake.IntakePistonRetractCommand;
import frc.robot.commands.intake.RunIntakeCommand;
import frc.robot.commands.magazine.MagazineRunCommand;
import frc.robot.commands.magazine.TimedMagazineRunCommand;
import frc.robot.commands.shooter.StartShooterCommand;
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
public class FourBallAutonomousCommand extends SequentialCommandGroup {
  /** Creates a new AutonomousCommandGroup. */
  public FourBallAutonomousCommand(TurretShooterSubsystem turretShooter, IntakeSubsystem intake, DriveSubsystem drive, MagazineSubsystem mag, TurretSpinnerSubsystem spinner) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetGyroCommand(drive),
      new StartShooterCommand(turretShooter, 0.0),
      
      //new ResetGyroCommand(drive),
      new IntakePistonExtendCommand(intake),
      new DriveUntilBallFoundCommand(drive, intake, mag, new DriveStraightCommand(drive, 0.35, 0.35), new WaitForTwoBallsThere()).withTimeout(35),
  //    new TurnTowardsHubCommand(drive),
      new TurnToAngleCommand(drive, 0.4, 180.0),
      
      new TurretHoodUpCommand(spinner),
      //new IntakePistonRetractCommand(intake),
    //  new DriveBySonarCommand(drive, 52.5),
      new StartShooterCommand(turretShooter, 1120),
      new WaitForRPMReachedCommand(),
      new TimedShootCommand(mag, intake, 1.5),
      new StartShooterCommand(turretShooter, 0.0),
      new TurnToAngleCommand(drive, 0.4, 40),
      new DriveBySonarCommand(drive, 40.0)

      //.alongWith(new RunIntakeCommand(intake), new MagazineRunCommand(mag, true)).until(MagazineSubsystem::areTwoBallsThere),
      //new RunIntakeCommand(intake),
      /*new WaitForTwoBallsThere(),
      //new DriveUntilBallFoundCommand(drive, intake, mag, new DriveBySonarCommand(drive, 25), new WaitForTwoBallsThere()),
      //new WaitForTwoBallsThere(),
      new TurnToAngleCommand(drive, 0.4, 180.0),
      new DriveBySonarCommand(drive, 44.0),
      new StartShooterCommand(turretShooter, 1120),
      new WaitForRPMReachedCommand(),
      new TimedShootCommand(mag, intake, 1.5),
      new StartShooterCommand(turretShooter, 0.0)*/
    );

    /*new StartShooterCommand(turretShooter, 650).andThen(
        new IntakePistonExtendCommand(intake),
       // new InstantCommand(() -> intake.intakePistonExtend(), intake),
        new DriveUntilBallFound(drive, magazine, intake),//.alongWith(new RunCommand(() -> { intake.intakeBigwheelOn(); intake.intakeMotorForward();}, intake)),
        new TurnTowardsHubCommand(drive, tapeVision),
        new WaitForRPMReachedCommand(),
        new TimedMagazineRunCommand(magazine,5.0));*/
  }
}
