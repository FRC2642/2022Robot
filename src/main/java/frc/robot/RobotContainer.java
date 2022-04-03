// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.SonarSubsystem;
import frc.robot.subsystems.TapeVisionSubsystem;
import frc.robot.subsystems.TurretShooterSubsystem;
import frc.robot.subsystems.TurretSpinnerSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.TwoBallAutonomousCommand;
import frc.robot.commands.AimAndShootCommand;
import frc.robot.commands.BallFollowerCommand;
import frc.robot.commands.DriveUntilBallFoundCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.TurnTowardsHubCommand;
import frc.robot.commands.intake.IntakePistonExtendCommand;
import frc.robot.commands.shooter.StartShooterCommand;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveSubsystem drive = new DriveSubsystem();
  public final TapeVisionSubsystem tapeVision = new TapeVisionSubsystem();
  public final VisionSubsystem ballVision = new VisionSubsystem();
  private final TurretShooterSubsystem turretShooter = new TurretShooterSubsystem();
  private final TurretSpinnerSubsystem turretSpinner = new TurretSpinnerSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final MagazineSubsystem magazine = new MagazineSubsystem();
  private final ClimberSubsystem climb = new ClimberSubsystem();
  public static final SonarSubsystem sonar = new SonarSubsystem();

  
  

 // private final Command ballFollowerCommand = new BallFollowerCommand(drive, ballVision);
  //private final SequentialCommandGroup ballIntaker = new BallFollowerIntakeCommand(intake, ballVision, drive, turretShooter, magazine, turretSpinner);

  public static XboxController driveController = new XboxController(0);
  public static XboxController auxController = new XboxController(1);

  public final Joystick rightDriveStick = new Joystick(1);
  
  private final Trigger auxLeftTrigger = new Trigger(magazine::getAuxLeftTrigger);
  private final Trigger auxRightTrigger = new Trigger(turretShooter::getAuxRightTrigger);

  private final Command turnTowardsHubCommand = new TurnTowardsHubCommand(drive);
  private final ParallelCommandGroup aimAndShoot = new AimAndShootCommand(drive, turretShooter);
 /* private final Command driveCommand = new DriveCommand(drive);
  private final Command intakePistonExtend = new IntakePistonExtendCommand(intake);
  private final Command intakePistonRetract = new IntakePistonRetractCommand(intake);
  private final Command intakeOutCommand = new IntakeOutCommand(intake);
  private final Command intakeOffCommand = new IntakeOffCommand(intake);
  private final Command tapeFollow = new TurnTowardsHubCommand(turretSpinner, tapeVision); */

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
      drive.move(-driveController.getRawAxis(1) * .90,(driveController.getRawAxis(0) * .80));
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

    
    new JoystickButton(auxController, Button.kStart.value)
    .whenPressed(new InstantCommand(() -> climb.climbPistonFoward(), climb));

    
    new JoystickButton(auxController, Button.kBack.value)
    .whenPressed(new InstantCommand(() -> climb.climbPistonBackward(), climb));


    //auto aim during tele-op
    new JoystickButton(driveController, Button.kA.value)
    .whenPressed(turnTowardsHubCommand);

    //auto aim and set shooter speed
    new JoystickButton(driveController, Button.kB.value)
    .whenPressed(aimAndShoot);



    /*new JoystickButton(auxController, Button.kB.value)
    .whenHeld(new RunCommand(climb::moveElevatorDown(0.5), climb));*/



    //preset shooting powers (change to speeds)
    /*new JoystickButton(driveController, Button.kX.value)
    .whileHeld(new RunCommand(() -> turretShooter.setSpeed(650), turretShooter));

    new JoystickButton(driveController, Button.kA.value)
    .whileHeld(new RunCommand(() -> turretShooter.setSpeed(1000), turretShooter));

    new JoystickButton(driveController, Button.kB.value)
    .whileHeld(new RunCommand(() -> turretShooter.setSpeed(1200), turretShooter));

    new JoystickButton(driveController, Button.kY.value)
    .whileHeld(new RunCommand(() -> turretShooter.setSpeed(3250), turretShooter));*/

    
    //figure out presets for eliminated backspin
    new POVButton(auxController, 0).whileHeld(new RunCommand(() -> turretShooter.setSpeed(650), turretShooter));
    new POVButton(auxController, 90).whileHeld(new RunCommand(() -> turretShooter.setSpeed(1200), turretShooter));
    new POVButton(auxController, 180).whileHeld(new RunCommand(() -> turretShooter.setSpeed(2000), turretShooter));
    new POVButton(auxController, 270).whileHeld(new RunCommand(() -> turretShooter.setSpeed(3250), turretShooter));





    new JoystickButton(auxController, 5)
    .whenHeld(new RunCommand(magazine::magReverse, magazine));



    SmartDashboard.putData("resetgyro",new ResetGyroCommand(drive));



    //not using climb pistons right now
    /*new JoystickButton(driveController, 6)
    .whenPressed(new InstantCommand(climb::climbPistonFoward));

    new JoystickButton(driveController, 5)
    .whenPressed(new InstantCommand(climb::climbPistonBackward));*/


    //new JoystickButton(driveController, Button.kX.value).whenPressed(intakePistonExtend);
    
    //new JoystickButton(driveController, Button.kY.value).whenPressed(intakePistonRetract);




        
        
    

  
  }

  //Gyro Methods
  
  
  public static boolean getDriveLeftTrigger(){
    double lTrigger = driveController.getLeftTriggerAxis();
    return (lTrigger > .5);

  }

  public static boolean getDriveRightTrigger(){
    double rTrigger = driveController.getRightTriggerAxis();
    return (rTrigger > .5);
  }

  public static boolean getJoystickData(){
    return (Math.abs(driveController.getRawAxis(0)) > 0.3);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    /*Command auto = new RunCommand(() -> turretShooter.setSpeed(1500), turretShooter).andThen
    (new RunCommand(() -> drive.move(-0.3,0),drive).withTimeout(1)).andThen(
      new RunCommand(() -> magazine.magRun()));*/

    //Command auto = new RunCommand(() -> drive.move(-0.3,0),drive).withTimeout(4).andThen(new RunCommand(() -> turretShooter.setSpeed(500), turretShooter));
  /*  Command auto = 
    new RunCommand(() -> drive.move(-0.4,0),drive).withTimeout(2).andThen(new
     RunCommand(()-> turretShooter.setSpeed(1500), turretShooter)).alongWith(
      new RunCommand(() -> intake.intakeBigwheelOn(), intake)).alongWith(
      new RunCommand(() -> magazine.magRun(), magazine));
//LMAO
    
    return new TimedDriveCommand(drive, 5);
    */


    //start shooter (will instantly finish), then drive backwards for 4 seconds at -0.3 speed. Then run the magazine when the rpm is reached on the turret.
     //new BallFollowerCommand(drive, ballVision);
    //  new TurnTowardsHubCommand(drive, tapeVision);
      /*new StartShooterCommand(turretShooter, 650).andThen(new TimedDriveCommand(drive, 1.0, 0.4)).andThen(new MagazineRunWhenRPMReachedCommand(magazine)).withTimeout(5).andThen(
        new TimedDriveCommand(drive, 3.0, -0.4))
      .alongWith(new RunCommand(() -> intake.intakeBigwheelOn(), intake)).alongWith(new InstantCommand(turretSpinner::turretHoodUp));*/

      Command auto =// new BallFollowerCommand(drive);
      
      //new IntakePistonExtendCommand(intake).andThen(new DriveUntilBallFoundCommand(drive, intake, magazine, new BallFollowerCommand(drive)));
      
      new TwoBallAutonomousCommand(turretShooter, intake, drive, magazine);//new InstantCommand(() -> intake.intakePistonExtend(), intake);

      // sets shooter speed to 1200 rpm, drives straight FORWARD with intake running until the 
      // lower light sensor senses a ball and then stops, searches for hub using tape vision pipeline
      // and stops when aimed, waits for shooter to reach rpm, and then runs magazine for 5 seconds
      
     //   .alongWith(
        //  new InstantCommand(() -> intake.intakePistonExtend(), intake).andThen(
       //   new RunCommand(() -> { intake.intakeBigwheelOn(); intake.intakeMotorForward();}, intake))
          //);

      //im so done with this bruh
    return auto;
  }
}
