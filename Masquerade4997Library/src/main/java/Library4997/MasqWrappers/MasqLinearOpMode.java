package Library4997.MasqWrappers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;

import Library4997.MasqResources.MasqUtils;
import Library4997.MasqSensors.MasqClock;


/**
 * Custom Linear opMode
 */
public abstract class MasqLinearOpMode extends LinearOpMode {
    protected DashBoard dash;
    protected MasqController controller1, controller2;
    protected MasqClock timeoutClock = new MasqClock();
    public final void runOpMode() throws InterruptedException {
        try {
            dash = new DashBoard(super.telemetry);
            dash.setNewFirst();
            MasqUtils.setLinearOpMode(this);
            controller1 = new MasqController(super.gamepad1, "controller1");
            controller2 = new MasqController(super.gamepad2, "controller2");
            runLinearOpMode();
        } finally {
            stopLinearOpMode();
        }
    }
    public abstract void runLinearOpMode() throws InterruptedException;
    public void stopLinearOpMode() {}
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
    public void sleep(double timeSeconds) {
        sleep((long) timeSeconds*1000);
    }
    public void sleep() {sleep(1000);}
}