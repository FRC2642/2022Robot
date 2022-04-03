// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class TimeBasedRamp {
    private Timer timer;
    private double seconds;
    private double deadband;
    public TimeBasedRamp(double secondsUntilFullPower, double deadband){
        timer = new Timer();
        this.seconds = secondsUntilFullPower;
        this.deadband = deadband;
        reset();
    }
    public double calculate(double process){
        timer.start();
        return MathR.proportion(timer.get(), deadband, seconds, 0.0, process);
    }
    public void reset(){
        timer.reset();
        timer.stop();
    }
}
