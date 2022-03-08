// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.drive.Vector2d;

/** Add your docs here. */
public class VectorR {
    public double Speed;
    public double Rotation;
    public RotationType type;
    public VectorR(double speed, double rotation, RotationType type){
        Speed = speed;
        Rotation = rotation;
        this.type = type;
    }
    public static VectorR fromVector2d(Vector2d vector){
        return new VectorR(Math.sqrt(vector.x * vector.x + vector.y * vector.y),Math.atan(vector.y/vector.x),RotationType.RADIANS);
    }
    enum RotationType {
        DEGREES,
        RADIANS
    }
}