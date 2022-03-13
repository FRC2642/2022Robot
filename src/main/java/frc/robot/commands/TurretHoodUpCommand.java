// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.SpinnerListModel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretShooterSubsystem;
import frc.robot.subsystems.TurretSpinnerSubsystem;

public class TurretHoodUpCommand extends CommandBase {
  TurretSpinnerSubsystem spinner;
  /** Creates a new TurretHoodUpCommand. */
  public TurretHoodUpCommand(TurretSpinnerSubsystem spinner) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.spinner = spinner;
    addRequirements(spinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spinner.turretHoodUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
