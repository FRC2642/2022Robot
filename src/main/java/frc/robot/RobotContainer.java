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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
import frc.robot.commands.BallFollowerIntakeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.IntakeOffCommand;
import frc.robot.commands.IntakeOutCommand;
import frc.robot.commands.IntakePistonExtendCommand;
import frc.robot.commands.IntakePistonRetractCommand;
import frc.robot.commands.MagazineRunCommand;
import frc.robot.commands.StartShooterCommand;
import frc.robot.commands.TimedDriveCommand;
import frc.robot.commands.TimedMagazineRunCommand;
import frc.robot.commands.TurnTowardsHubCommand;
import frc.robot.commands.WaitForRPMReachedCommand;
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
  private final SequentialCommandGroup ballIntaker = new BallFollowerIntakeCommand(intake, ballVision, drive, turretShooter, magazine, turretSpinner);

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
  private final Command tapeFollow = new TurnTowardsHubCommand(turretSpinner, tapeVision);

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
    /*turretSpinner.setDefaultCommand(
      new RunCommand(
        () -> turretSpinner.manuelTurnTurret(
          auxController.getRawAxis(4) * 0.40
          ), turretSpinner
          ));*/

    
    intake.setDefaultCommand(
      new RunCommand(
        () -> {if (intake.getLeftTrigger()){
          //intake.intakePistonRetract();
          intake.intakeBigwheelOn();
          intake.intakeMotorForward();
          magazine.magReverse();
          }
          else{
            //intake.intakePistonExtend();
            intake.intakeMotorOff();
            intake.intakeBigwheelOff();
            magazine.magStop();
          }
          }, intake));
      
    //intake.setDefaultCommand(intakeOffCommand);

          
   
    climb.setDefaultCommand(
      new RunCommand(
        () -> climb.climberStop()
        ,climb
    ));


    drive.setDefaultCommand(new RunCommand(() ->{ 
    if(getDriveRightTrigger()){
      drive.move(-driveController.getRawAxis(1) * .90,(driveController.getRawAxis(0) * .90));
    }
    //slower turn, fast straight
    else if(getDriveLeftTrigger()){
      drive.move(-driveController.getRawAxis(1) * .50,(driveController.getRawAxis(0) * .50));
    }
    //normal drive
    else{
      drive.move(-driveController.getRawAxis(1) * .6,(driveController.getRawAxis(0) * .60));
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
    auxRightTrigger.whileActiveContinuous(new RunCommand(() -> turretShooter.setSpeed(1500), turretShooter));
    
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


    //climber up and down
    new JoystickButton(auxController, Button.kX.value)
    .whileHeld(new RunCommand(() -> climb.moveElevator(0.60), climb));

    new JoystickButton(auxController, Button.kB.value)
    .whileHeld(new RunCommand(() -> climb.moveElevatorDown(-0.60), climb));


    /*new JoystickButton(auxController, Button.kB.value)
    .whenHeld(new RunCommand(climb::moveElevatorDown(0.5), climb));*/



    //preset shooting powers (change to speeds)
    new JoystickButton(driveController, Button.kX.value)
    .whileHeld(new RunCommand(() -> turretShooter.setSpeed(650), turretShooter));

    new JoystickButton(driveController, Button.kA.value)
    .whileHeld(new RunCommand(() -> turretShooter.setSpeed(1000), turretShooter));

    new JoystickButton(driveController, Button.kB.value)
    .whileHeld(new RunCommand(() -> turretShooter.setSpeed(1200), turretShooter));

    new JoystickButton(driveController, Button.kY.value)
    .whileHeld(new RunCommand(() -> turretShooter.setSpeed(3250), turretShooter));



    new JoystickButton(auxController, 5)
    .whenHeld(new RunCommand(magazine::magReverse, magazine));






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
    //return ballIntaker;
    //return tapeFollow;
    /*Command auto = new RunCommand(() -> turretShooter.setSpeed(1500), turretShooter).andThen
    (new RunCommand(() -> drive.move(-0.3,0),drive).withTimeout(1)).andThen(
      new RunCommand(() -> magazine.magRun()));*/

    //Command auto = new RunCommand(() -> drive.move(-0.3,0),drive).withTimeout(4).andThen(new RunCommand(() -> turretShooter.setSpeed(500), turretShooter));
  /*  Command auto = 
    new RunCommand(() -> drive.move(-0.4,0),drive).withTimeout(2).andThen(new
     RunCommand(()-> turretShooter.setSpeed(1500), turretShooter)).alongWith(
      new RunCommand(() -> intake.intakeBigwheelOn(), intake)).alongWith(
      new RunCommand(() -> magazine.magRun(), magazine));
    */


    //start shooter (will instantly finish), then drive backwards for 4 seconds at -0.3 speed. Then run the magazine when the rpm is reached on the turret.
    Command auto = 
      /*new StartShooterCommand(turretShooter, 650).andThen(new TimedDriveCommand(drive, 1.0, 0.4)).andThen(new MagazineRunWhenRPMReachedCommand(magazine)).withTimeout(5).andThen(
        new TimedDriveCommand(drive, 3.0, -0.4))
      .alongWith(new RunCommand(() -> intake.intakeBigwheelOn(), intake)).alongWith(new InstantCommand(turretSpinner::turretHoodUp));*/

      new StartShooterCommand(turretShooter, 1500)
      .andThen(new WaitForRPMReachedCommand())
      .andThen(new TimedMagazineRunCommand(magazine,3.0))
      .andThen(new DriveDistanceCommand(drive, 5.0, -0.4))
      .alongWith(new RunCommand(() -> intake.intakeBigwheelOn(), intake));

      
    return auto;
  }
}
