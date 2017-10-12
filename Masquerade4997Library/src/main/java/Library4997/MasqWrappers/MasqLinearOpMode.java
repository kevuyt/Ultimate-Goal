package Library4997.MasqWrappers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Library4997.MasqRobot;

/**
 * Custom Linear opMode
 */

public abstract class MasqLinearOpMode extends LinearOpMode {
    protected DashBoard dash;
    public MasqRobot robot;
    protected MasqController controller1, controller2;
    protected MasqControllerV2 controller1V2, controller2V2;
    public final void runOpMode() throws InterruptedException {
        try {
            robot = MasqRobot.getInstance(this);
            dash = new DashBoard(super.telemetry);
            dash.setNewFirst();
            controller1 = new MasqController(super.gamepad1, "controller1");
            controller2 = new MasqController(super.gamepad2, "controller2");
            controller1V2 = new MasqControllerV2(super.gamepad1, "controller1");
            controller2V2 = new MasqControllerV2(super.gamepad2, "controller2");
            run();
        } finally {
            stopLinearOpMode();
        }
    }
    public abstract void run() throws InterruptedException;
    public void stopLinearOpMode() {}
}