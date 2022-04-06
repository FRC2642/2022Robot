// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.waitfor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SonarSubsystem;

public class WaitForSonarReached extends CommandBase {
  double setpoint;
  /** Creates a new WaitForSonarReached. */
  public WaitForSonarReached(double setpoint) {
    this.setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return Math.abs(SonarSubsystem.getSonarDistance() - setpoint) < 5.0;
  }
}
