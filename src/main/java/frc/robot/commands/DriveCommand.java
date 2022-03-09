package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

//Starts driving indefinitely
public class DriveCommand extends CommandBase {

  private DriveSubsystem drive;
  private double speed = 0.25;
  private double turn = 0;
  
  public DriveCommand(DriveSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
    SmartDashboard.putNumber("speed", speed);
    SmartDashboard.putNumber("turn", turn);
  }
  

  @Override
  public void execute() {
    var s = SmartDashboard.getNumber("speed", speed);
    var d = SmartDashboard.getNumber("turn", turn);
    drive.move(s, d);
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