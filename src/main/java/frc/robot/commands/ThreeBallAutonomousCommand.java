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
public class ThreeBallAutonomousCommand extends SequentialCommandGroup {
  /** Creates a new AutonomousCommandGroup. */
  public ThreeBallAutonomousCommand(TurretShooterSubsystem turretShooter, IntakeSubsystem intake, DriveSubsystem drive, MagazineSubsystem mag) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //extend intake and drive until second ball found
      new IntakePistonExtendCommand(intake),
      new DriveUntilBallFoundCommand(drive, intake, mag, new DriveSpeedCommand(drive, 0.4, 0.0), new WaitForTwoBallsThere()),
      //turn towards hub and drive til 44in. away and shoot at 1100 rpm wait a sec and shoot again
      new TurnTowardsHubCommand(drive),
      new DriveBySonarCommand(drive, 44.0),
      new StartShooterCommand(turretShooter, 1100),
      new WaitForRPMReachedCommand(),
      new TimedShootCommand(mag, intake, 1.0),
      new WaitForRPMReachedCommand(),
      new TimedShootCommand(mag, intake, 1.0),
      //search for third ball and drive until picked up
      new BallFollowerCommand(drive),
      new DriveUntilBallFoundCommand(drive, intake, mag, new DriveSpeedCommand(drive, 0.4, 0.0), new WaitForOneBallThere()),
      //turn towards hub and shoot
      new TurnTowardsHubCommand(drive),
      new DriveBySonarCommand(drive, 44.0),
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
