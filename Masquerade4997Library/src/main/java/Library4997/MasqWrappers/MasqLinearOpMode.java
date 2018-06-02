package Library4997.MasqWrappers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Library4997.MasqUtilities.MasqUtils;
import SubSystems4997.MasqRobot;


/**
 * Custom Linear opMode
 */
public abstract class MasqLinearOpMode extends LinearOpMode {
    protected DashBoard dash;
    public MasqRobot robot;
    protected MasqController controller1, controller2;
    public final void runOpMode() throws InterruptedException {
        try {
            dash = new DashBoard(super.telemetry);
            dash.setNewFirst();
            MasqUtils.setLinearOpMode(this);
            controller1 = new MasqController(super.gamepad1, "controller1");
            controller2 = new MasqController(super.gamepad2, "controller2");
            runLinearOpMode();
        }
        finally {
            stopLinearOpMode();
        }
    }
    public abstract void runLinearOpMode() throws InterruptedException;
    public void stopLinearOpMode() {}
    public void runSimultaneously(Runnable r1, Runnable r2) {
        Thread t1 = new Thread(r1);
        Thread t2 = new Thread(r2);
        t1.start();
        t2.start();
        while (opModeIsActive() && (t1.isAlive() || t2.isAlive())) {
            idle();
        }
    }
    public void sleep(int time) {
        try {
            Thread.sleep((long) time * 1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
    public void sleep(double time) {
        try {
            Thread.sleep((long) time);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
    public void sleep() {sleep(MasqUtils.DEFAULT_SLEEP_TIME);}
}