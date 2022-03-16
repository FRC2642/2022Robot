// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.TurretShooterSubsystem;

public class MagazineRunWhenRPMReachedCommand extends CommandBase {
  MagazineSubsystem mag;
  /** Creates a new MagazineRunCommand. */
  public MagazineRunWhenRPMReachedCommand(MagazineSubsystem mag) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mag = mag;
    addRequirements(mag);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  Timer magTimer = null;

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    if (TurretShooterSubsystem.isCloseToSetRPM()) {
      mag.magRun();
<<<<<<< HEAD
      if (magTimer == null) {
        magTimer = new Timer();
        magTimer.start();
      }
    }
    else {
      mag.magStop();
      
      //cancel the timer
      magTimer = null;
=======
>>>>>>> accb991ae487f142e718f97e1a82d5c2abc0b500
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mag.magStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return magTimer != null && magTimer.get() > Constants.RUN_MAG_TIME;
  }
}
