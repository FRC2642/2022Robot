// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.TurretShooterSubsystem;

public class TimedMagazineRunCommand extends CommandBase {
  MagazineSubsystem mag;
  Timer timer;
  double time;
  /** Creates a new MagazineRunCommand. */
  public TimedMagazineRunCommand(MagazineSubsystem mag, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mag = mag;
    this.timer = new Timer();
    this.time = time;
    addRequirements(mag);
    
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
      mag.magRun();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mag.magStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > time;
  }
}
