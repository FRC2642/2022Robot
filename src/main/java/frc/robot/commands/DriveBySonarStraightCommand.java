// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.drive.DriveWithPIDCommand;
import frc.robot.commands.waitfor.WaitForSonarReached;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveBySonarStraightCommand extends ParallelRaceGroup {
  /** Creates a new DriveBySonarStraightCommand. */
  public DriveBySonarStraightCommand(DriveSubsystem drive, double setpoint) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveWithPIDCommand(drive, 0.35),
      new WaitForSonarReached(44.0)
    );
  }
}
