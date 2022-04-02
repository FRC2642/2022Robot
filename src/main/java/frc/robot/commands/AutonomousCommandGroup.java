// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveBySonarCommand;
import frc.robot.commands.drive.DriveSpeedCommand;
import frc.robot.commands.intake.IntakePistonExtendCommand;
import frc.robot.commands.magazine.TimedMagazineRunCommand;
import frc.robot.commands.shooter.StartShooterCommand;
import frc.robot.commands.waitfor.WaitForOneBallThere;
import frc.robot.commands.waitfor.WaitForRPMReachedCommand;
import frc.robot.commands.waitfor.WaitForTwoBallsThere;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.TurretShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousCommandGroup extends SequentialCommandGroup {
  /** Creates a new AutonomousCommandGroup. */
  public AutonomousCommandGroup(TurretShooterSubsystem turretShooter, IntakeSubsystem intake, DriveSubsystem drive, MagazineSubsystem mag) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new StartShooterCommand(turretShooter, 0.0),
      new IntakePistonExtendCommand(intake),
      new DriveUntilBallFoundCommand(drive, intake, mag, new DriveSpeedCommand(drive, 0.4, 0.0), new WaitForTwoBallsThere()),
      new TurnTowardsHubCommand(drive),
      new DriveBySonarCommand(drive, 44.0),
      new StartShooterCommand(turretShooter, 1100),
      new WaitForRPMReachedCommand(),
      new TimedShootCommand(mag, intake, 1.0),
      new WaitForRPMReachedCommand(),
      new TimedShootCommand(mag, intake, 1.0),
      new StartShooterCommand(turretShooter, 0),
      new DriveUntilBallFoundCommand(drive, intake, mag, new BallFollowerCommand(drive), new WaitForOneBallThere()),
      new TurnTowardsHubCommand(drive),
      new DriveBySonarCommand(drive, 44.0),
      new StartShooterCommand(turretShooter, 1100),
      new WaitForRPMReachedCommand(),
      new TimedShootCommand(mag, intake, 1.0),
      new StartShooterCommand(turretShooter, 0)
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
