// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import javax.swing.border.EtchedBorder;

import edu.wpi.first.wpilibj.drive.Vector2d;

/** Wrapper class for vectors */
public class VectorR {

    //vector data
    private final Vector2d translation;

    public double getX(){
        return translation.x;
    }
    public double getY(){
        return translation.y;
    }
    public double getAngle(){
        return Math.atan2(translation.y,translation.x);
    }
    public double getMagnitude(){
        return Math.sqrt(Math.pow(translation.x,2) + Math.pow(translation.y,2));
    }

    //constructor
    private VectorR(){
        translation = new Vector2d();
    }

    //helper functions

    //adds a vector to this vector
    public void add(VectorR vector){
        translation.x += vector.getX();
        translation.y += vector.getY();
    }
    public void sub(VectorR vector){
        translation.x -= vector.getX();
        translation.y -= vector.getY();
    }
    public void setFromPolar(double distance, double angle){
        translation.x = distance * Math.cos(angle);
        translation.y = distance * Math.sin(angle);
    }
    public void setFromCartesian(double x, double y){
        translation.x = x;
        translation.y = y;
    }
    public static VectorR fromPolar(double distance, double angle) {
        VectorR v = new VectorR();
        v.setFromPolar(distance, angle);
        return v;
    }
    public static VectorR fromCartesian(double x, double y) {
        VectorR v = new VectorR();
        v.setFromCartesian(x, y);
        return v;
    }
}