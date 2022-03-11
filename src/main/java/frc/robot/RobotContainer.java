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
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.TapeVisionSubsystem;
import frc.robot.subsystems.TurretShooterSubsystem;
import frc.robot.subsystems.TurretSpinnerSubsystem;
import frc.robot.commands.BallFollowerCommand;
import frc.robot.commands.BallFollowerIntakeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeOffCommand;
import frc.robot.commands.IntakeOutCommand;
import frc.robot.commands.IntakePistonExtendCommand;
import frc.robot.commands.IntakePistonRetractCommand;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem drive = new DriveSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();
  public final TapeVisionSubsystem tapeVision = new TapeVisionSubsystem();
  private final TurretShooterSubsystem turretShooter = new TurretShooterSubsystem();
  private final TurretSpinnerSubsystem turretSpinner = new TurretSpinnerSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final MagazineSubsystem magazine = new MagazineSubsystem();
  private final ClimberSubsystem climb = new ClimberSubsystem();
  

  private final Command ballFollowerCommand = new BallFollowerCommand(drive, vision);
  private final SequentialCommandGroup ballIntaker = new BallFollowerIntakeCommand(intake, vision, drive);

  public static XboxController driveController = new XboxController(0);
  public static XboxController auxController = new XboxController(1);

  public final Joystick rightDriveStick = new Joystick(1);
  
  private final Trigger auxLeftTrigger = new Trigger(magazine::getAuxLeftTrigger);
  private final Trigger auxRightTrigger = new Trigger(turretShooter::getAuxRightTrigger);

  private final Command driveCommand = new DriveCommand(drive);
  private final Command intakePistonExtend = new IntakePistonExtendCommand(intake);
  private final Command intakePistonRetract = new IntakePistonRetractCommand(intake);
  private final Command intakeOutCommand = new IntakeOutCommand(intake);
  private final Command intakeOffCommand = new IntakeOffCommand(intake);


  //public final Button driveButtonX = new JoystickButton(driveController, Constants.xButtonDrive);
  //public final Button driveButtonB = new JoystickButton(driveController, Constants.bButtonDrive); 



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    
    /*drive.setDefaultCommand(
      new RunCommand(
        () -> drive.move(
       -driveController.getRawAxis(1) * 0.6,
        driveController.getRawAxis(0) *0.6
          ), drive
    ));*/ 

    //sicko and slow modes (check if this works, if not go back to above drive command)
    //drive.setDefaultCommand(driveCommand); 


    turretShooter.setDefaultCommand(
      new RunCommand(
        () -> 
        turretShooter.stop(), turretShooter
          ));

  magazine.setDefaultCommand(
            new RunCommand(
              () -> 
              magazine.magStop(), magazine
                ));

    //with limit switches
    turretSpinner.setDefaultCommand(
      new RunCommand(
        () -> turretSpinner.manuelTurnTurret(
          auxController.getRawAxis(4) * 0.40
          ), turretSpinner
          ));

    
    intake.setDefaultCommand(
      new RunCommand(
        () -> {if (intake.getLeftTrigger()){
          //intake.intakePistonRetract();
          intake.intakeBigwheelOn();
          intake.intakeMotorForward();
          }
          else{
            //intake.intakePistonExtend();
            intake.intakeMotorOff();
            intake.intakeBigwheelOff();
          }
          }, intake));
      
    //intake.setDefaultCommand(intakeOffCommand);

          
   
    climb.setDefaultCommand(
      new RunCommand(
        () -> climb.moveElevator(
         -auxController.getRawAxis(5)
        ), climb
    ));


    drive.setDefaultCommand(new RunCommand(() ->{ 
    if(getDriveLeftTrigger()){
      drive.move(-driveController.getRawAxis(1) * .80,(driveController.getRawAxis(0) * .80));
    }
    //slower turn, fast straight
    else if(getDriveRightTrigger()){
      drive.move(-driveController.getRawAxis(1) * .70,(driveController.getRawAxis(0) * .30));
    }
    //normal drive
    else{
      drive.move(-driveController.getRawAxis(1) * .6,(driveController.getRawAxis(0) * .45));
    }
  },drive));



    


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {


    
    //runs magazine
    auxLeftTrigger.whileActiveContinuous(new RunCommand(() -> magazine.magRun(), magazine));

    //runs shooter (need to figure out speed)
    auxRightTrigger.whileActiveContinuous(new RunCommand(() -> turretShooter.setSpeed(0.43), turretShooter));
    
    //changes turret hood
    new JoystickButton(auxController, Button.kY.value)
    .whenPressed(new InstantCommand(turretSpinner::turretHoodUp));

    new JoystickButton(auxController, Button.kA.value)
    .whenPressed(new InstantCommand(turretSpinner::turretHoodDown));

    //check these bumper values, I'm not sure which is which
    new JoystickButton(driveController, 6)
    .whenPressed(new InstantCommand(intake::intakePistonExtend, intake));

    new JoystickButton(driveController, 5)
    .whenPressed(new InstantCommand(intake::intakePistonRetract, intake));



    //preset shooting powers (change to speeds)
    new JoystickButton(driveController, Button.kA.value)
    .whileHeld(new RunCommand(() -> turretShooter.setSpeed(0.80), turretShooter));

    new JoystickButton(driveController, Button.kB.value)
    .whileHeld(new RunCommand(() -> turretShooter.setSpeed(0.60), turretShooter));

    new JoystickButton(driveController, Button.kX.value)
    .whileHeld(new RunCommand(() -> turretShooter.setSpeed(0.50), turretShooter));

    new JoystickButton(driveController, Button.kY.value)
    .whileHeld(new RunCommand(() -> turretShooter.setSpeed(0.70), turretShooter));






    //not using climb pistons right now
    /*new JoystickButton(driveController, 6)
    .whenPressed(new InstantCommand(climb::climbPistonFoward));

    new JoystickButton(driveController, 5)
    .whenPressed(new InstantCommand(climb::climbPistonBackward));*/


    //new JoystickButton(driveController, Button.kX.value).whenPressed(intakePistonExtend);
    
    //new JoystickButton(driveController, Button.kY.value).whenPressed(intakePistonRetract);




        
        
    

  
  }
  public static boolean getDriveLeftTrigger(){
    double lTrigger = driveController.getLeftTriggerAxis();
    return (lTrigger > .5);

  }

  public static boolean getDriveRightTrigger(){
    double rTrigger = driveController.getRightTriggerAxis();
    return (rTrigger > .5);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return ballIntaker;
  }
}
