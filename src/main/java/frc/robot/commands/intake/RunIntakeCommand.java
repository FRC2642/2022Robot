// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeCommand extends CommandBase {
  IntakeSubsystem intake;
  /** Creates a new IntakeOn. */
  public RunIntakeCommand(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //feed bigwheel speed every 22ms
    intake.intakeMotorForward();
    intake.intakeBigwheelOn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    intake.intakeMotorOff();
    intake.intakeBigwheelOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
