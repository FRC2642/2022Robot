// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TapeVisionSubsystem;
import frc.robot.subsystems.TurretSpinnerSubsystem;
import frc.robot.subsystems.TapeVisionSubsystem;
public class TurretScanCommand extends CommandBase {

  public boolean turnRight = false;
  
  TurretSpinnerSubsystem turn;
  TapeVisionSubsystem vision;
  /** Creates a new TurretScanCommand. */
  public TurretScanCommand(TurretSpinnerSubsystem turn, TapeVisionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turn = turn;
    this.vision = vision; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
    
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (turnRight == false){
      turn.turnTurret(-.2);
    } else {
        turn.turnTurret(.2);
    } 
    if (turn.clockwiseSwitchOn()) {
        turnRight = false;
    } else if (turn.counterClockwiseSwitchOn()) {
      turnRight = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if (vision.hubInFrame(new Vector2d(), new Vector2d()) == TapeVisionSubsystem.HubInFrameReason.DETECTED){
      return true;
    }else{
      return false;
    }
    
  }
}
