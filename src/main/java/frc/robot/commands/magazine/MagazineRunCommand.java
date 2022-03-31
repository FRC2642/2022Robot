// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.magazine;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MagazineSubsystem;

public class MagazineRunCommand extends CommandBase {
  MagazineSubsystem mag;
  boolean down;
  /** Creates a new MagazineRunCommand. */
  public MagazineRunCommand(MagazineSubsystem mag, boolean down) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mag = mag;
    this.down = down;
    addRequirements(mag);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (down) mag.magReverse();
    else      mag.magRun();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mag.magStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
