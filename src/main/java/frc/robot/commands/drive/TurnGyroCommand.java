// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import javax.swing.plaf.metal.MetalComboBoxUI.MetalPropertyChangeListener;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MathR;

public class TurnGyroCommand extends CommandBase {
  DriveSubsystem drive;
  double angle;
  /** Creates a new TurnGyroCommand. */
  public TurnGyroCommand(DriveSubsystem drive, double angle) {
    this.drive = drive;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.move(0.0, MathR.limit((DriveSubsystem.getYaw()-angle)/90, -0.35, 0.35));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
