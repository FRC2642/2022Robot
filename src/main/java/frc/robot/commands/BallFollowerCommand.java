// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class BallFollowerCommand extends CommandBase {
  DriveSubsystem drive;
  VisionSubsystem vision;
  double error;
  /** Creates a new BallFollowerCommand. */
  public BallFollowerCommand(DriveSubsystem drive, VisionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.vision = vision;

    addRequirements(drive,vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    
    //implement PID?
    if(vision.getCenterX() < 70){ //left
      drive.arcadeDrive(0, -1.0); //(0, -0.4)
    }
    else if(vision.getCenterX() > 90){ //right

      drive.arcadeDrive(0, 1.0); //(0, 0.4)
    }
    else{
      drive.arcadeDrive(0, 0);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
