// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ResetEncoderCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.ResetVectorCommand;
import frc.robot.commands.TimedShootCommand;
import frc.robot.commands.drive.DriveAtFixedHeadingCommand;
import frc.robot.commands.drive.DriveBySonarCommand;
import frc.robot.commands.drive.DriveSpeedCommand;
import frc.robot.commands.drive.DriveStraightCommand;
import frc.robot.commands.drive.TurnToAngleCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.intake.IntakePistonExtendCommand;
import frc.robot.commands.intake.IntakePistonRetractCommand;
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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneBallAutonomousCommand extends SequentialCommandGroup {
  /** Creates a new AutonomousCommandGroup. */
  public OneBallAutonomousCommand(TurretShooterSubsystem turretShooter, IntakeSubsystem intake, DriveSubsystem drive, MagazineSubsystem mag) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //extend intake and drive until second ball found
      new ResetEncoderCommand(),
      new ResetGyroCommand(),
      new ResetVectorCommand(7.0,true),
      new StartShooterCommand(turretShooter, 1500),
      new WaitForRPMReachedCommand(),
      new TimedShootCommand(mag, intake, 1),
      new StartShooterCommand(turretShooter, 0.0),
      new DriveSpeedCommand(drive, -0.4, 0.0).withTimeout(3)
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
