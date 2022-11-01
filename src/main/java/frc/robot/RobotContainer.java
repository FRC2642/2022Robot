// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.Vector;

//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.PixySubsystem;
import frc.robot.subsystems.SonarSubsystem;
import frc.robot.subsystems.TapeVisionSubsystem;
import frc.robot.subsystems.TurretShooterSubsystem;
import frc.robot.subsystems.TurretSpinnerSubsystem;
import frc.robot.subsystems.VectorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.auto.FourBallAutonomousCommand;
import frc.robot.commands.auto.OneBallAutonomousCommand;
import frc.robot.commands.auto.ThreeBallAutonomousCommand;
import frc.robot.commands.auto.TwoBallAutonomousCommand;
import frc.robot.commands.drive.DriveDistanceCommand;
import frc.robot.commands.drive.TurnToAngleCommand;
import frc.robot.commands.drive.TurnTowardsHubUsingVectorsCommand;
import frc.robot.commands.AimAndShootCommand;
import frc.robot.commands.BallFollowerCommand;
import frc.robot.commands.ClimbTiltPistonOneCommand;
import frc.robot.commands.ClimbTiltPistonTwoCommand;
import frc.robot.commands.DriveUntilBallFoundCommand;
import frc.robot.commands.InterruptSubsystemsCommand;
import frc.robot.commands.ResetEncoderCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.TurnTowardsHubCommand;

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
  public final VectorSubsystem vectors = new VectorSubsystem();
 // public final VectorSubsystem vector = new VectorSubsystem();
 // public final PixySubsystem pixy = new PixySubsystem();

  
  

 // private final Command ballFollowerCommand = new BallFollowerCommand(drive, ballVision);
  //private final SequentialCommandGroup ballIntaker = new BallFollowerIntakeCommand(intake, ballVision, drive, turretShooter, magazine, turretSpinner);

  public static XboxController driveController = new XboxController(0);
  public static XboxController auxController = new XboxController(1);

  public final Joystick rightDriveStick = new Joystick(1);
  
  private final Trigger auxLeftTrigger = new Trigger(magazine::getAuxLeftTrigger);
  private final Trigger auxRightTrigger = new Trigger(turretShooter::getAuxRightTrigger);

  private final Command turnTowardsHubCommand = new TurnTowardsHubCommand(drive);
  private final AimAndShootCommand aimAndShoot = new AimAndShootCommand(drive, turretShooter, magazine, intake);
  private final Command climbTiltPistonOne = new ClimbTiltPistonOneCommand(climb);
  private final Command climbTiltPistonTwo = new ClimbTiltPistonTwoCommand(climb);


  SendableChooser<Command> chooser = new SendableChooser<>();

 /* private final Command driveCommand = new DriveCommand(drive);
  private final Command intakePistonExtend = new IntakePistonExtendCommand(intake);
  private final Command intakePistonRetract = new IntakePistonRetractCommand(intake);
  private final Command intakeOutCommand = new IntakeOutCommand(intake);
  private final Command intakeOffCommand = new IntakeOffCommand(intake);
  private final Command tapeFollow = new TurnTowardsHubCommand(turretSpinner, tapeVision); */

  //public final Button driveButtonB = new JoystickButton(driveController, Constants.bButtonDrive); 



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   /* vector.backLeftMotor = drive.backLeft;
    vector.backRightMotor = drive.backRight;
    vector.pigeon2 = drive.pigeon2; */
    // Configure the button bindings
    configureButtonBindings();

    chooser.addOption("One-Ball Auto", new OneBallAutonomousCommand(turretShooter, intake, drive, magazine));
    chooser.addOption("Two-Ball Auto", new TwoBallAutonomousCommand(turretShooter, intake, drive, magazine, turretSpinner));
    chooser.addOption("Three-Ball Auto", new ThreeBallAutonomousCommand(turretShooter, intake, drive, magazine, turretSpinner));
    chooser.addOption("Four-Ball Auto", new FourBallAutonomousCommand(turretShooter, intake, drive, magazine, turretSpinner));

    SmartDashboard.putData("Auto Chooser", chooser);

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
          
   
    climb.setDefaultCommand(
      new RunCommand(
        () -> climb.climberStop()
        ,climb
    ));

   

    drive.setDefaultCommand(new RunCommand(() -> { 
    if(getDriveRightTrigger()){
      drive.configDriveRamp(0.4);
      drive.move(-driveController.getRawAxis(1) * .90,(driveController.getRawAxis(0) * .80));
    }
    //slower turn, fast straight
    else if(getDriveLeftTrigger()){
      drive.configDriveRamp(0.30);
      drive.move(-driveController.getRawAxis(1) * .50,(driveController.getRawAxis(0) * .50));
    }
    //normal drive
    else{
      drive.configDriveRamp(0.35);
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
    auxLeftTrigger.whileActiveContinuous(new RunCommand(() -> magazine.magRun(), magazine).alongWith(new RunCommand(() -> intake.intakeBigwheelOn(), intake)));

    //runs shooter (need to figure out speed)
    auxRightTrigger.whileActiveContinuous(new RunCommand(() -> turretShooter.setSpeed(5600), turretShooter));
    
    //changes turret hood
    new JoystickButton(auxController, Button.kY.value)
    .whenPressed(new InstantCommand(turretSpinner::turretHoodUp));

    new JoystickButton(auxController, Button.kA.value)
    .whenPressed(new InstantCommand(turretSpinner::turretHoodDown));

    //***********************INTAKE & MAGAZINE BUTTONS***********************/
    //extends and retracts intake piston
    new JoystickButton(driveController, 6)
    .whenPressed(new InstantCommand(intake::intakePistonExtend, intake));

    new JoystickButton(driveController, 5)
    .whenPressed(new InstantCommand(intake::intakePistonRetract, intake));


    //new JoystickButton(driveController, Button.kX.value).whenPressed(climbTiltPistonOne);



    



    //auto aim and set shooter speed
    /*new JoystickButton(driveController, Button.kB.value)
    .whenPressed(aimAndShoot);*/



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
    //up close, low hub, hood come up automatically


    //runs magazine and big wheel together
    /*auxLeftTrigger.whileActiveContinuous(new RunCommand(() -> magazine.magRun(), magazine).alongWith(
      new RunCommand(() -> intake.intakeBigwheelOn(), intake)
    ));*/

    //reverse magazine
    new JoystickButton(auxController, 5)
    .whenHeld(new RunCommand(magazine::magReverse, magazine));


  
    
    //***********************CLIMBER BUTTONS***********************/
    //tilt the climber back
    new JoystickButton(driveController, Button.kBack.value)
    .whenPressed(new InstantCommand(() -> climb.climbPistonFoward(), climb));
    
    new JoystickButton(driveController, Button.kStart.value)
    .whenPressed(new InstantCommand(() -> climb.climbPistonBackward(), climb));

    //move climb rack up and down
    new JoystickButton(auxController, Button.kB.value)
    .whileHeld(new RunCommand(() -> climb.moveElevator(), climb));
    
    new JoystickButton(auxController, Button.kX.value)
    .whileHeld(new RunCommand(() -> climb.moveElevatorDown(), climb));

    //toggle high and low pressure for climbing
    new POVButton(driveController, 180).whenPressed(climbTiltPistonOne);
    new POVButton(driveController, 0).whenPressed(climbTiltPistonTwo);



    //***********************SHOOTER BUTTONS***********************/    
    //changes turret hood
    new JoystickButton(auxController, Button.kY.value)
    .whenPressed(new InstantCommand(turretSpinner::turretHoodUp));
    
    new JoystickButton(auxController, Button.kA.value)
    .whenPressed(new InstantCommand(turretSpinner::turretHoodDown));
    
    //up close, low hub
    new POVButton(auxController, 0).whileHeld(new RunCommand(() -> turretShooter.setSpeed(900), turretShooter));
    //up close, high hub
    new POVButton(auxController, 90).whileHeld(new RunCommand(() -> turretShooter.setSpeed(2000), turretShooter));
    //on the line shot, high hub
    new POVButton(auxController, 180).whileHeld(new RunCommand(() -> turretShooter.setSpeed(1500), turretShooter));
    //from launch pad (safe zone, far shot)
    
    new POVButton(auxController, 270).whileHeld(new RunCommand(() -> turretShooter.setSpeed(2500), turretShooter));



    //***********************EXTRA***********************/    
    //interrupts all commands running
    SmartDashboard.putData("interrupt", new InterruptSubsystemsCommand(drive, turretShooter, magazine, intake, climb));
    SmartDashboard.putData("reset encoder", new ResetEncoderCommand());
    


    //auto aim during tele-op
    new JoystickButton(driveController, Button.kA.value)
    .whenPressed(new TurnTowardsHubUsingVectorsCommand(drive, 0.4));
    
  }

  
  
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
    //return new FourBallAutonomousCommand(turretShooter, intake, drive, magazine, turretSpinner);
    //return new ThreeBallAutonomousCommand(turretShooter, intake, drive, magazine);
    return chooser.getSelected();
  }
}
