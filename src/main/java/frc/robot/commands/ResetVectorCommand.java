// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.Vector;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VectorSubsystem;

public class ResetVectorCommand extends CommandBase {
  double robotDistanceFromHubPlacedOnGround;
  boolean robotIsFacingHub;
  /** Creates a new ResetVectorCommand. */
  public ResetVectorCommand(double robotDistanceFromHubPlacedOnGround, boolean robotIsFacingHub) {
    this.robotDistanceFromHubPlacedOnGround = robotDistanceFromHubPlacedOnGround;
    this.robotIsFacingHub = robotIsFacingHub;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    VectorSubsystem.robotPlacedOnGroundDistanceFromHub = robotDistanceFromHubPlacedOnGround;
    VectorSubsystem.robotPlacedFacingTowardsHub = robotIsFacingHub;
    VectorSubsystem.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
