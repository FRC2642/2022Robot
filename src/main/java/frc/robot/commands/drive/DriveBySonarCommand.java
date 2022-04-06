// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SonarSubsystem;
import frc.robot.utils.MathR;

public class DriveBySonarCommand extends CommandBase {
  private DriveSubsystem drive;
  private double distance;
  private double setpoint;
  /** Creates a new DriveBySonarCommand. */
  public DriveBySonarCommand(DriveSubsystem drive, double distance) {
    this.drive = drive;
    this.distance = distance;
    
    drive.setPIDCoefficients(0.2, 0, 0);
     addRequirements(drive);// here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    setpoint = DriveSubsystem.getYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turn = MathR.limit(drive.calculatePID(DriveSubsystem.getYaw(), setpoint), -1.0, 1.0);
    
    drive.move(MathR.limit((SonarSubsystem.getSonarDistance() - distance)/20,-0.40,0.40),turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(SonarSubsystem.getSonarDistance() - distance) < 5.0;
  }
}
