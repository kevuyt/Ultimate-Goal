package Library4997.MasqWrappers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Library4997.MasqExternal.MasqAnnotations.MasqLinearOpModeClass;
import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqRobot;

/**
 * Custom Linear opMode
 */
@MasqLinearOpModeClass
public abstract class MasqLinearOpMode extends LinearOpMode {
    protected DashBoard dash;
    public MasqRobot robot;
    protected MasqController controller1, controller2;
    public final void runOpMode() throws InterruptedException {
        try {
            robot = new MasqRobot();
            dash = new DashBoard(super.telemetry);
            dash.setNewFirst();
            MasqExternal.setLinearOpMode(this);
            controller1 = new MasqController(super.gamepad1, "controller1");
            controller2 = new MasqController(super.gamepad2, "controller2");
            runLinearOpMode();
        } finally {
            stopLinearOpMode();
        }
    }
    public abstract void runLinearOpMode() throws InterruptedException;
    public void stopLinearOpMode() {}
    public void runSimultaneously(Runnable r1, Runnable r2) throws InterruptedException {
        Thread t1 = new Thread(r1);
        Thread t2 = new Thread(r2);
        t1.start();
        t2.start();
        while (opModeIsActive() && (t1.isAlive() || t2.isAlive())) {
            idle();
        }
    }
}