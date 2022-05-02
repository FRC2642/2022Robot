// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.MathR;

public class BallFollowerCommand extends CommandBase {
  DriveSubsystem drive;
  double error;
  double setpoint;
  double rotationValue;
  /** Creates a new BallFollowerCommand. */
  public BallFollowerCommand(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.drive = drive;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    
   /* if(vision.getCenterX() <= 30){ //left
      System.out.println("Left");
      drive.move(0, 0.3);
      SmartDashboard.putNumber("CenterX", vision.getCenterX());
    }
    else if (vision.getCenterX() >= 130){
      System.out.println("Right");
      drive.move(0, -0.3);
      SmartDashboard.putNumber("CenterX", vision.getCenterX());
    }
    else{
      System.out.println("PID Movement");
      setpoint = 80;
      SmartDashboard.putNumber("CenterX", vision.getCenterX());
      rotationValue = drive.calculatePID(vision.getCenterX(), setpoint);
      SmartDashboard.putNumber("RotationValue", rotationValue);
      if (rotationValue > 1){
        rotationValue = 1;
      }
      else if(rotationValue < -1){
        rotationValue = -1;
      }
      drive.move(0, rotationValue * -0.3);
    }*/
   /* if(vision.getCenterX() < 70){ //left
      drive.move(0.0, -0.35); //(0, -0.4)
    }
    else if(vision.getCenterX() > 90){ //right
      drive.move(0.0, 0.35); //(0, 0.4)
    }
    else{
      drive.move(0,0.0);
    }*/
    double turn = MathR.proportion(VisionSubsystem.getCenterX()-80, 0.27, 80, 10, 0.32);//MathR.limit((VisionSubsystem.getCenterX()-80)/80,-0.4,0.4);
//    SmartDashboard.putNumber("turn", turn);
    drive.move(0.0, turn);
    //drive.move(turn < 0.2 ? 0.35 : 0,turn < 0.2 ? 0 : turn);
  //  if (VisionSubsystem.getCenterY() > 80 || !VisionSubsystem.isDetection()) drive.move(turn == 0.0 ? 0.35 : 0.0,turn);
  //  else drive.move(0.35,0.0);

  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(VisionSubsystem.getCenterX() - 80) < 10;
  }
}
