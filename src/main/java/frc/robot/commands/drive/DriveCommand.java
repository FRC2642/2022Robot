// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
  /** Creates a new DriveCommand. */
  DriveSubsystem drive;
  public DriveCommand(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.getDriveLeftTrigger()){
      drive.move(-RobotContainer.driveController.getRawAxis(1) * .80,(RobotContainer.driveController.getRawAxis(0) * .80));
    }
    //slower turn, fast straight
    else if(RobotContainer.getDriveRightTrigger()){
      drive.move(-RobotContainer.driveController.getRawAxis(1) * .70,(RobotContainer.driveController.getRawAxis(0) * .30));
    }
    //normal drive
    else{
      drive.move(-RobotContainer.driveController.getRawAxis(1) * .6,(RobotContainer.driveController.getRawAxis(0) * .45));
    }

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
