// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.crypto.spec.DHPrivateKeySpec;
import javax.swing.plaf.synth.SynthScrollBarUI;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
//import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.BallVisionSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.TapeVisionSubsystem;
import frc.robot.subsystems.TurretShooterSubsystem;
import frc.robot.subsystems.TurretSpinnerSubsystem;
import frc.robot.commands.BallFollowerCommand;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem drive = new DriveSubsystem();
  public final TapeVisionSubsystem tapeVision = new TapeVisionSubsystem();
  public final BallVisionSubsystem ballVision = new BallVisionSubsystem();
  private final TurretShooterSubsystem turretShooter = new TurretShooterSubsystem();
  private final TurretSpinnerSubsystem turretSpinner = new TurretSpinnerSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final MagazineSubsystem magazine = new MagazineSubsystem();
  private final ClimberSubsystem climb = new ClimberSubsystem();
  

  private final Command ballFollowerCommand = new BallFollowerCommand(drive, ballVision);

  public static XboxController driveController = new XboxController(0);
  public static XboxController auxController = new XboxController(1);

  public final Joystick rightDriveStick = new Joystick(1);

  public final JoystickButton A_Button = new JoystickButton(driveController, Button.kA.value);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    
    drive.setDefaultCommand(
      new RunCommand(
        
        () -> drive.move(
          
          driveController.getRawAxis(0) * 0.6,
          driveController.getRawAxis(1) * 0.6
          ), drive
    ));

    turretShooter.setDefaultCommand(
      new RunCommand(
        () -> 
        turretShooter.setSpeed(
          driveController.getRightTriggerAxis()
          ), turretShooter
          ));

    magazine.setDefaultCommand(
            new RunCommand(
              () -> 
              magazine.magStop(), magazine
                ));

    //has no limits (just for testing purposes)
    turretSpinner.setDefaultCommand(
      new RunCommand(
        () -> turretSpinner.turnTurret(
          driveController.getRawAxis(4) * 0.20
          ), turretSpinner
          ));

    
    intake.setDefaultCommand(
      new RunCommand(
        () -> {if (intake.getLeftTrigger()){
          intake.intakeBigwheelOn();
          intake.intakeMotorForward();
          }
          else{
            intake.intakeMotorOff();
            intake.intakeBigwheelOff();
          }
          }, intake)

          
    );
    climb.setDefaultCommand(
      new RunCommand(
        () -> climb.moveElevator(
         rightDriveStick.getRawAxis(0)
        ), climb
    ));

    


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {


    A_Button.whenHeld(
      new RunCommand(() -> magazine.magRun(),
      magazine));

    /*leftTrigger.whileActiveContinuous(
      new RunCommand(() -> {intake.intakeMotorForward();
        intake.intakeBigwheelOn();}, intake));*/
        //leftTrigger.whileActiveContinuous(new RunCommand(intake.intakeMotorForward()), intake);
        
        
    

  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return ballFollowerCommand;
  }
}
