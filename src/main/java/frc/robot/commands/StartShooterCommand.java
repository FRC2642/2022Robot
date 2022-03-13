// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretShooterSubsystem;

public class StartShooterCommand extends CommandBase {
  /** Creates a new ShooterCommand. */
  TurretShooterSubsystem shooter;
  double rpm;
  public StartShooterCommand(TurretShooterSubsystem shooter, double rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.rpm = rpm;
    addRequirements(shooter);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setSpeed(rpm);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //we instantly finish because the shooter uses a closed loop PID and can run without being called constantly
    return true;
  }
}
