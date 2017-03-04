package BasicLib4997;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import BasicLib4997.MasqMotors.MasqRobot.MasqRobot;

/**
 * Custom Linear opMode
 */

public abstract class MasqLinearOpMode extends LinearOpMode {

    protected DashBoard dash;
    public MasqRobot robot;
    public Controller controller1;
    public Controller controller2;
    public final void runOpMode() throws InterruptedException {
        try {
            dash = new DashBoard(super.telemetry);
            dash.setNewFirst();
            robot = new MasqRobot(super.telemetry);
            controller1 = new Controller(super.gamepad1);
            controller2 = new Controller(super.gamepad2);
            runLinearOpMode();
        } finally {
            stopLinearOpMode();
        }
    }

    public abstract void runLinearOpMode() throws InterruptedException;
    public void stopLinearOpMode() {
    }

}