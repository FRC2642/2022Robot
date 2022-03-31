// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;

public class DriveUntilBallFound extends CommandBase {
  DriveSubsystem drive;
  MagazineSubsystem mag;
  IntakeSubsystem intake;
  /** Creates a new DriveUntilBallFound. */
  public DriveUntilBallFound(DriveSubsystem drive, MagazineSubsystem mag, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.mag = mag;
    this.intake = intake;
    
    addRequirements(drive, mag);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.move(0.4, 0.0);
    mag.magReverse();
    intake.intakeMotorForward();
    intake.intakeBigwheelOn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    intake.intakeBigwheelOff();
    intake.intakeMotorOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mag.areTwoBallsThere();
  }
}
