// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BallFollowerCommand;
import frc.robot.commands.IntakePistonExtendCommand;
import frc.robot.commands.IntakeSpinForwardCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.IntakePistonRetractCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BallFollowerIntakeCommand extends SequentialCommandGroup {
  /** Creates a new BallFollowerIntakeCommand. */
  IntakeSubsystem intake;
  VisionSubsystem vision;
  DriveSubsystem drive;
  
  public BallFollowerIntakeCommand(IntakeSubsystem intake, VisionSubsystem vison, DriveSubsystem drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());


    this.intake = intake;
    this.vision = vison;
    this.drive = drive;

    addCommands(new IntakePistonExtendCommand(intake),
                new IntakeSpinForwardCommand(intake),
                new BigWheelMove(intake),
                new BallFollowerCommand(drive, vision));
  }
}
