package MasqLibrary.MasqResources;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

import static MasqLibrary.MasqResources.MasqUtils.*;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public abstract class MasqLinearOpMode extends LinearOpMode {
    public DashBoard dash;
    protected ElapsedTime timeoutClock = new ElapsedTime();

    public final void runOpMode() {
        try {
            dash = new DashBoard(super.telemetry);
            dash.setNewFirst();
            setLinearOpMode(this);
            setDash(dash);
            runLinearOpMode();
        } finally {
            stopLinearOpMode();
        }
    }
    public abstract void runLinearOpMode();
    public void stopLinearOpMode() {}

    public void runSimultaneously(Runnable... actions) {
        List<Thread> threads = new ArrayList<>();
        int i = 0;
        for (Runnable action : actions) {
            threads.add(new Thread(action));
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