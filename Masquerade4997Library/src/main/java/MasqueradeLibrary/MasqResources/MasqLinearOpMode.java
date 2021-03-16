package MasqueradeLibrary.MasqResources;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;

import static MasqueradeLibrary.MasqResources.MasqUtils.*;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public abstract class MasqLinearOpMode extends LinearOpMode {
    public DashBoard dash;
    protected MasqClock timeoutClock = new MasqClock();
    public final void runOpMode() {
        try {
            dash = new DashBoard(super.telemetry);
            dash.setNewFirst();
            setLinearOpMode(this);
            runLinearOpMode();
        } finally {
            stopLinearOpMode();
        }
    }
    public abstract void runLinearOpMode();
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
    public void sleep() {sleep(DEFAULT_SLEEP_TIME);}
    public Gamepad getDefaultController() {return gamepad1;}
}