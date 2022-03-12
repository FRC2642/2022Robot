// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Vector;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TapeVisionSubsystem;
import frc.robot.subsystems.TurretSpinnerSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TurnTowardsHubCommand extends CommandBase {
  TurretSpinnerSubsystem turn;
  TapeVisionSubsystem vision;
  double setpoint;
  double rotationValue;
  /** Creates a new AimAtRetroReflectiveCommand. */
  public TurnTowardsHubCommand(TurretSpinnerSubsystem turn, TapeVisionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turn = turn;
    this.vision = vision;
    this.setpoint = 0;
    addRequirements(turn, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Vector2d center = new Vector2d();
    Vector2d size = new Vector2d();
    SmartDashboard.putString("hub finder", vision.hubInFrame(center, size).toString());
    SmartDashboard.putString("hub", "center x: " + center.x + "center y: " + center.y);

     if(center.x <= 50){ //left
      System.out.println("Left");
      turn.turnTurret(-0.4);
      SmartDashboard.putNumber("CenterX", center.x);
    }
    else if (center.x >= 110){
      System.out.println("Right");
      turn.turnTurret(0.4);
      SmartDashboard.putNumber("CenterX", center.x);
    }
    else{
      System.out.println("PID Movement");
      setpoint = 80;
     
      rotationValue = turn.calculatePID(center.x, setpoint);
      
      if (rotationValue > 1){
        rotationValue = 1;
      }
      else if(rotationValue < -1){
        rotationValue = -1;
      }
      turn.turnTurret(rotationValue * 0.5);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
