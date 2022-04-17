// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.VectorSubsystemOld;

public class FindVectorDistanceCommand extends CommandBase {
  VectorSubsystemOld vector;
  double encoderValue;
  double totalEncoderValue;
  double vectorComponentX;
  double vectorComponentY;
  double distance;
  /** Creates a new FindVectorDistanceCommand. */
  public FindVectorDistanceCommand(VectorSubsystemOld vector) {
    this.vector = vector;
    addRequirements(vector);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    vector.vectorComponentX += (Math.sin(180 * (Math.PI/180)) * Constants.ENCODER_TICKS_FOUR_FEET);
    vector.vectorComponentY += (Math.cos(180 * (Math.PI/180)) * Constants.ENCODER_TICKS_FOUR_FEET);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPulses;
    currentPulses = vector.getEncoderDistanceAverage();
    encoderValue = currentPulses - vector.lastEncoderPulses;
    vector.lastEncoderPulses = currentPulses;
    vector.vectorComponentX += (Math.sin(vector.heading() * (Math.PI/180)) * encoderValue);
    vector.vectorComponentY += (Math.cos(vector.heading() * (Math.PI/180)) * encoderValue);
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
