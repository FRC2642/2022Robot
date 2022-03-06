// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Constructor;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class TapeVisionSubsystem extends SubsystemBase {

  private final ArrayList<Vector2d> tapeContourPositions = new ArrayList<Vector2d>();
  public static final int NUM_TAPE_ON_HUB = 6;
  /** Creates a new VisionSubsystem. */
  //all vision (tape and ball) go in here (change variable names as necessary)
  public TapeVisionSubsystem() {}

  public ArrayList<Vector2d> getDetections(){
    return tapeContourPositions;
  }
  public void clearDetections(){
    tapeContourPositions.clear();
  }
  public void addDetection(Vector2d contourPosition){
    tapeContourPositions.add(contourPosition);
  }

  //vision processing
  public HubInFrameReason hubInFrame(Vector2d hubPosition, Vector2d hubSizeInFrame) throws Exception {

    if (hubPosition == null || hubSizeInFrame == null) throw new Exception("you failed.");
    if (tapeContourPositions.size() < NUM_TAPE_ON_HUB) return HubInFrameReason.NOT_ENOUGH_TAPES;

    //FIND HUB FROM POINTS
    //convert points to array
    Vector2d[] contours = new Vector2d[tapeContourPositions.size()];
    double[] cache = new double[tapeContourPositions.size()];
    for(int i = 0; i < tapeContourPositions.size(); ++i){
      contours[i] = tapeContourPositions.get(i);
    }
    //sort by nearest neighbor
    for (int i = 0; i < contours.length; ++i) {
      Object[] s = getClosestInSetFrom(contours, i); //<<< this is broken
      contours[i] = contours[(int)s[0]];
      cache[i] = (double)s[1];
    }
    //take derivative of distances
    for (int i = 1; i < cache.length; ++i){
      cache[i] = cache[i] - cache[i - 1];
    }
    //find groups
    ArrayList<Double[]> sets = new ArrayList<Double[]>();
    double[] current = new double[2];
    for (int i = 1; i < cache.length; ++i){
      //left off here
    }

    return HubInFrameReason.NOT_DETECTED;
  }
  private Object[] getClosestInSetFrom(Vector2d[] contours, int index){
      double smallestD = Double.MAX_VALUE;
      int smallestI = 0;
      for (int i = index + 1; i < contours.length; ++i){
        double d = Math.sqrt(contours[i].x*contours[i].x + contours[index].y*contours[index].y);
        if (d < smallestD) {
          smallestD = d;
          smallestI = i;
        }
      }
      return new Object[] { smallestI, smallestD };
  }
  


  enum HubInFrameReason{
    DETECTED,
    NOT_DETECTED,
    NOT_ENOUGH_TAPES
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

