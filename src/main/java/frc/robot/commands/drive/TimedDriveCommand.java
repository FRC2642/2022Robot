// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TimedDriveCommand extends CommandBase {
  /** Creates a new TimedDriveCommand. */
  DriveSubsystem drive;
  Timer timer;
  Double seconds;
  Double speed;

  public TimedDriveCommand(DriveSubsystem drive, double seconds, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.seconds = seconds;
    this.speed = speed;
    timer = new Timer();
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.move(speed,0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= seconds;
  }
}
