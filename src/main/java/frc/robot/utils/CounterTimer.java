package frc.robot.utils;

import java.util.Timer;
public class CounterTimer {
    Timer timer;
    public boolean timeEnded = false;
    public void activateTimer() throws InterruptedException{
        timer.wait(3000);
        timeEnded = true;
    }
    public void stopTimer(){
        timer.cancel();
        timeEnded = false;
    }
}
