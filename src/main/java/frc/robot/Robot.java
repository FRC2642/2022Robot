// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

//import com.kauailabs.navx.frc.AHRS;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.vision.BlurContour;
import frc.robot.vision.RetroReflectivePipeline;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
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
  public VisionThread tapeVisionThread;

  public static final Object imgLock = new Object();
  public Rect rect = new Rect();
  public boolean isSquare;


  
  
  //public static PowerDistribution pdh =  new PowerDistribution(0, PowerDistribution.ModuleType.kRev);



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
    intakecam.setFPS(60);
    intakecam.setResolution(320, 240);    //160X120
    intakecam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    //turret camera setup
    turretcam = CameraServer.startAutomaticCapture(1);
    turretcam.setFPS(60);
    turretcam.setResolution(320, 240);    //160X120
    turretcam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    

    //pdh.clearStickyFaults();
    double[] hue = null;
    double[] sat = null;
    double[] lum = null;
    
    var alliance = DriverStation.getAlliance();
    if (alliance == Alliance.Blue) {
      hue = Constants.HSL_HUE_BLUE;
      sat = Constants.HSL_SAT_BLUE;
      lum = Constants.HSL_LUM_BLUE;
    }
    else if (alliance == Alliance.Red){
      hue = Constants.HSL_HUE_RED;
      sat = Constants.HSL_SAT_RED;
      lum = Constants.HSL_LUM_RED;
    }
    //vision thread to look for red balls
    redBallVisionThread = new VisionThread(intakecam, new BlurContour(hue, sat, lum), pipeline -> {
      if (!pipeline.filterContoursOutput().isEmpty()) {
          Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
          synchronized (imgLock) {
              rect = r;
              //centerX = 2*r.x + r.width - (320/2);
              //centerY = 2*r.y + r.height - (240/2);
              if (Math.abs(rect.width - rect.height) < Constants.MIN_NUM_PIXELS_RECT_SIMILARITY){
                isSquare = true;
                m_robotContainer.ballVision.setCenterX(r.x +(0.5*r.width));
                m_robotContainer.ballVision.setCenterY(r.y +(0.5*r.height));
              }
              else { 
                isSquare = false;
              }
          }
      }
    });
  
    redBallVisionThread.start();

     tapeVisionThread = new VisionThread(turretcam, new RetroReflectivePipeline(), pipeline -> {
      m_robotContainer.tapeVision.clearDetections();
      for (var contour : pipeline.filterContoursOutput()) {
          Rect r = Imgproc.boundingRect(contour);
          synchronized (imgLock) {

                var rX = r.x +(0.5*r.width);
                var rY = r.y +(0.5*r.height);

                m_robotContainer.tapeVision.addDetection(new Vector2d(rX, rY));
              }
        }
    });
  
    tapeVisionThread.start();
   // redBallVisionThread.stop();
    //redBallVisionThread.stop(); (how do i get it to stop?)
    m_robotContainer.drive.resetEncoder();


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

    SmartDashboard.putNumber("width", rect.width); 
    SmartDashboard.putNumber("height", rect.height);
    SmartDashboard.putBoolean("is square",isSquare);
   // SmartDashboard.putNumber("navx", Robot.navx.getYaw());

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
