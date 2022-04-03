// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TapeVisionSubsystem;
import frc.robot.utils.MathR;

public class TurnTowardsHubCommand extends CommandBase {

  private DriveSubsystem drive;
  private Timer timer = new Timer();
  /** Creates a new TurnTowardsHubCommand. */
  public TurnTowardsHubCommand(DriveSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    TapeVisionSubsystem.resetCenterX();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if (centerX < 70)       drive.drive(0,-0.4);
    else if (centerX > 90)  drive.drive(0,0.4);
    else                    drive.drive(0,0);*/
    drive.move(0,MathR.limit(TapeVisionSubsystem.getNormalizedCenterX()/2,-0.39,0.39));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*if ((Math.abs(TapeVisionSubsystem.getNormalizedCenterX()) < 0.1)){
      return true;
    }
    else if (RobotContainer.getJoystickData()){
      return true;
    }
    else{
      return false;
    }*/

    return (Math.abs(TapeVisionSubsystem.getNormalizedCenterX()) < 0.1);
  }
}
