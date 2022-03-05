// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public static UsbCamera intakecam;
  public static UsbCamera turretcam;
  //public VideoSink camServer;
  public VisionThread redBallVisionThread;

  public static double centerX = 0.0;
  public static double centerY = 0.0;
  public static double targetArea = 0.0;
  public static final Object imgLock = new Object();
  public Rect rect = new Rect();
  public boolean isSquare;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
    //intake camera setup
    intakecam = CameraServer.startAutomaticCapture(0);
    intakecam.setFPS(10);
    intakecam.setResolution(320, 240);    //160X120
    intakecam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    //turret camera setup
    turretcam = CameraServer.startAutomaticCapture(0);
    turretcam.setFPS(10);
    turretcam.setResolution(320, 240);    //160X120
    turretcam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    

    //vision thread to look for red balls
    redBallVisionThread = new VisionThread(intakecam, new BlurContour(Constants.HSL_HUE_RED, Constants.HSL_SAT_RED, Constants.HSL_LUM_RED), pipeline -> {
      if (!pipeline.filterContoursOutput().isEmpty()) {
          Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
          synchronized (imgLock) {
              rect = r;
              //centerX = 2*r.x + r.width - (320/2);
              //centerY = 2*r.y + r.height - (240/2);
              if (Math.abs(rect.width - rect.height) < Constants.MIN_NUM_PIXELS_RECT_SIMILARITY){
                isSquare = true;
                centerX = r.x +(0.5*r.width);
                centerY = r.y +(0.5*r.height);
              }
              else { 
                isSquare = false;
              }
              targetArea = r.area();
          }
      }
    });
  
    redBallVisionThread.start();
    //redBallVisionThread.stop(); (how do i get it to stop?)

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    SmartDashboard.putNumber("centerX", centerX);
    SmartDashboard.putNumber("width", rect.width); 
    SmartDashboard.putNumber("height", rect.height);
    SmartDashboard.putBoolean("is square",isSquare);

    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
