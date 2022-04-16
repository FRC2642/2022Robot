// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
      //extend intake and drive until second ball found
      new ResetEncoderCommand(drive),
      new ResetGyroCommand(drive),
      new TurretHoodUpCommand(spinner),
      new StartShooterCommand(turretShooter, 1100),
      new WaitForRPMReachedCommand(),
      new TimedShootCommand(mag, intake, 1.5),
      new StartShooterCommand(turretShooter, 0.0),
      new TurnToAngleCommand(drive, 0.4, 190),
      new BallFollowerCommand(drive).withTimeout(2.0),
      //new IntakePistonExtendCommand(intake),
      //new DriveUntilBallFoundCommand(drive, intake, mag, new DriveStraightCommand(drive, 0.375, 0.45), new WaitForOneBallThere().withTimeout(35)),
      new DriveDistanceCommand(drive, 2.7, 0.45, 0.3),
      //new IntakeOutCommand(intake).until(MagazineSubsystem::getLowerLightSensor),
      //new IntakeSpinForwardCommand(intake).until(MagazineSubsystem::getLowerLightSensor),
      new IntakePistonExtendCommand(intake),
      //new DriveUntilBallFoundCommand(drive, intake, mag, new DriveSpeedCommand(drive, 0.32, 0.0), new WaitForOneBallThere()),
      new DriveSpeedCommand(drive, 0.3, 0.0).alongWith(new RunIntakeCommand(intake)).until(MagazineSubsystem::isOneBallThere).withTimeout(3.0),
      new IntakePistonRetractCommand(intake),
      new TurnToAngleCommand(drive, 0.4, 95).alongWith(new TimedShootCommand(mag, intake, 1)),
      
      new BallFollowerCommand(drive).withTimeout(2.0),
     // new TimedShootCommand(mag, intake, 1),
      //new DriveUntilBallFoundCommand(drive, intake, mag, new DriveStraightCommand(drive, 0.40, 0.45), new WaitForTwoBallsThere().withTimeout(35)),
      new DriveDistanceCommand(drive, 4.75, 0.45, 0.3),
      new IntakePistonExtendCommand(intake),
      
      new DriveSpeedCommand(drive, 0.3, 0.0).alongWith(new RunIntakeCommand(intake), new MagazineRunCommand(mag, true)).until(MagazineSubsystem::areTwoBallsThere).withTimeout(3.0),
   //   new DriveUntilBallFoundCommand(drive, intake, mag, new DriveSpeedCommand(drive, 0.32, 0.0), new WaitForTwoBallsThere()),
      new IntakePistonRetractCommand(intake),
      
      new StartShooterCommand(turretShooter, 1000),
      new TurnToAngleCommand(drive, 0.45, -15.5),
      
      new TurretHoodUpCommand(spinner),
      //new DriveBySonarCommand(drive, 56.5),
      new WaitForRPMReachedCommand(),
      new TimedShootCommand(mag, intake, 3.5)
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
