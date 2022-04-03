// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.TurretShooterSubsystem;

public class ShootUntilBallGoneCommand extends CommandBase {
  /** Creates a new ShooterCommand. */
  TurretShooterSubsystem shooter;
  double rpm;
  Timer timer = new Timer();

  public ShootUntilBallGoneCommand(TurretShooterSubsystem shooter, double rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.rpm = rpm;
    addRequirements(shooter);
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
    shooter.setSpeed(rpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setSpeed(0.0);
  }
  


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (MagazineSubsystem.getLowerLightSensor() == false && MagazineSubsystem.getUpperLightSensor() == false) {
      return true;
    }
    else if (RobotContainer.getJoystickData()){
      return true;
    }
    else{
      return false;
    }
  }
}
