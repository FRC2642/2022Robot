package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

//Starts driving indefinitely
public class DriveCommand extends CommandBase {

  private DriveSubsystem drive;
  
  public DriveCommand(DriveSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
  }
  

  @Override
  public void execute() {
    drive.arcadeDrive(0.25, 0);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}