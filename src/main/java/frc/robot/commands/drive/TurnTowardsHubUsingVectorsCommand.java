// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.Vector;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VectorSubsystem;

public class TurnTowardsHubUsingVectorsCommand extends TurnToAngleCommand {
  /** Creates a new TurnTowardsHubUsingVectorsCommand. */
  public TurnTowardsHubUsingVectorsCommand(DriveSubsystem drive, double turnSpeed) {
    super(drive, turnSpeed, 0.0);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setPIDCoefficients(0.075, 0, 0);
    angle = VectorSubsystem.getAngleToHub();
  }
}
