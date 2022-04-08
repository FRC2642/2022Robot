// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.w3c.dom.css.RGBColor;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.DataStreamFilter;
import frc.robot.vision.BlurContour;
import frc.robot.vision.RGBPipeline;

public class VisionSubsystem extends SubsystemBase {
  private UsbCamera camera;
  private VisionThread visionthread;
  private Rect rect;
  private boolean isSquare = false;

  DataStreamFilter fps = new DataStreamFilter(10);

  private double centerX = 0;
  private double centerY = 0;
  private boolean isDetection = false;

  private Timer frametimer = new Timer();

  public static final Object imgLock = new Object();
  private static VisionSubsystem instance;
  /** Creates a new TapeVisionSubsystem. */
  public VisionSubsystem() {
    instance = this;
   /* camera = CameraServer.startAutomaticCapture(0);

    camera.setFPS(15);
    camera.setResolution(320, 240);
    camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    
   // SmartDashboard.putNumber("ball cam fps actual?", camera.getActualFPS()); 

        //pdh.clearStickyFaults();
    double[] r = null;
    double[] g = null;
    double[] b = null;
    
    var alliance = DriverStation.getAlliance();
    if (alliance == Alliance.Blue) {
      r = Constants.RGB_BLUE_rgbThresholdRed;
      g = Constants.RGB_BLUE_rgbThresholdGreen;
      b = Constants.RGB_BLUE_rgbThresholdBlue;
      
   /*   hue = Constants.HSL_HUE_BLUE;
      sat = Constants.HSL_SAT_BLUE;
      lum = Constants.HSL_LUM_BLUE; 
    }
    else if (alliance == Alliance.Red){
      
      r = Constants.RGB_RED_rgbThresholdRed;
      g = Constants.RGB_RED_rgbThresholdGreen;
      b = Constants.RGB_RED_rgbThresholdBlue;
 /*     hue = Constants.HSL_HUE_RED;
      sat = Constants.HSL_SAT_RED;
      lum = Constants.HSL_LUM_RED; 
    }
    frametimer.start();
    frametimer.reset();
    visionthread = new VisionThread(camera, new RGBPipeline(r,g,b), pipeline -> {
      SmartDashboard.putNumber("ball vision fps", fps.calculate( 1.0/frametimer.get()));
      frametimer.reset();
      if (!pipeline.findContoursOutput().isEmpty()) {
          Rect re = Imgproc.boundingRect(pipeline.findContoursOutput().get(0));
          synchronized (imgLock) {
              rect = re;
              //centerX = 2*r.x + r.width - (320/2);
              //centerY = 2*r.y + r.height - (240/2);
              if (Math.abs(rect.width - rect.height) < Constants.MIN_NUM_PIXELS_RECT_SIMILARITY){
                isSquare = true;
                
              
              centerX = re.x +(0.5*re.width);
              centerY = re.y +(0.5*re.height);

              isDetection = true;
              }
              else { 
                isSquare = false;
                isDetection= false;
                centerX = 0.0;
                
              }


          }
      }
    });
  
    visionthread.start();*/
  }

  //vision methods
  public static double getCenterX(){
    return instance.centerX;
  }
  public static double getCenterY(){
    return instance.centerY;
  }
  public static boolean isDetection(){
    return instance.isDetection;
  }

  @Override
  public void periodic() {
    //SmartDashboard.putBoolean("issquare", isSquare);
    //SmartDashboard.putNumber("center ball x", getCenterX());
    //SmartDashboard.putNumber("center ball y", centerY);
    //SmartDashboard.putBoolean("is detection", isDetection());
    // This method will be called once per scheduler run
  }
}
