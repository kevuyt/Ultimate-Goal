package Library4997.MasqResources;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;

import Library4997.MasqUtils;
import Library4997.MasqSensors.MasqClock;


/**
 * Custom Linear opMode
 */
public abstract class MasqLinearOpMode extends LinearOpMode {
    public DashBoard dash;
    protected MasqClock timeoutClock = new MasqClock();
    public final void runOpMode() throws InterruptedException {
        try {
            dash = new DashBoard(super.telemetry);
            dash.setNewFirst();
            MasqUtils.setLinearOpMode(this);
            runLinearOpMode();
        } finally {
            stopLinearOpMode();
        }
    }
    public abstract void runLinearOpMode() throws InterruptedException;
    public void stopLinearOpMode() throws InterruptedException {}
    public void runSimultaneously(Runnable... runnables) {
        List<Thread> threads = new ArrayList<>();
        int i = 0;
        for (Runnable runnable : runnables) {
            threads.add(new Thread(runnable));
            threads.get(i).start();
            i++;
        }
        int count = 0;
        while (opModeIsActive() && count < i) {
            count = 0;
            for(Thread t : threads) if (!t.isAlive()) count++;
        }
    }
    public void sleep(double timeSeconds) {MasqUtils.sleep(timeSeconds);}
    public void sleep() {MasqUtils.sleep();}
    public Gamepad getDefaultController() {return gamepad1;}
}