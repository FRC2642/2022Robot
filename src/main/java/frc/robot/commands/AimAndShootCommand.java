// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeSpinForwardCommand;
import frc.robot.commands.magazine.MagazineRunCommand;
import frc.robot.commands.shooter.ShootUntilBallGoneCommand;
import frc.robot.commands.shooter.StartShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.TurretShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimAndShootCommand extends SequentialCommandGroup {
  /** Creates a new AimAndShootCommand. */
  //DriveSubsystem drive;

  public AimAndShootCommand(DriveSubsystem drive, TurretShooterSubsystem shooter, MagazineSubsystem mag, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new StartShooterCommand(shooter, 1100.0),
                new TurnTowardsHubCommand(drive),
                new TimedShootCommand(mag, intake, 1.0),
                new StartShooterCommand(shooter, 0.0));
  }
}
