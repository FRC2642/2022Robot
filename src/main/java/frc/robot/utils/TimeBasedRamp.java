// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;


import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class TimeBasedRamp {
    private Timer timer;
    private double secondsUntilFullPower;
    private double expectedFullPower;
    private double deadband;
    public TimeBasedRamp(double secondsUntilFullPower, double expectedFullPower, double deadband){
        timer = new Timer();
        this.secondsUntilFullPower = secondsUntilFullPower;
        this.deadband = deadband;
        this.expectedFullPower = expectedFullPower;
        reset();
    }
    public double calculate(double process){
        timer.start();
        return MathR.proportion(Math.signum(process) * timer.get(), deadband, secondsUntilFullPower * (Math.abs(process)/expectedFullPower), 0.0, Math.abs(process));
    }
    public void reset(){
        timer.reset();
        timer.stop();
    }
}
