// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSpinnerSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TurnTowardsHubCommand extends CommandBase {
  TurretSpinnerSubsystem turn;
  VisionSubsystem vision;
  /** Creates a new AimAtRetroReflectiveCommand. */
  public TurnTowardsHubCommand(TurretSpinnerSubsystem turn, VisionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turn = turn;
    this.vision = vision;

    addRequirements(turn, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(vision.getCenterX() < 70){
      turn.turnTurret(-.5);
    }
    else if(vision.getCenterX() > 90) {
      turn.turnTurret(.5);
    }
    else{
      turn.turnTurret(0);
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
