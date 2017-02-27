package BasicLib4997;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import BasicLib4997.MasqMotors.MasqRobot.MasqRobot;

/**
 * Custom Linear opMode
 */

public abstract class MasqLinearOpMode extends LinearOpMode {

    protected DashBoard dash;
    public MasqRobot robot;
    public final void runOpMode() throws InterruptedException {
        try {
            dash = new DashBoard(super.telemetry);
            dash.setNewFirst();
            runLinearOpMode();
        } finally {
            stopLinearOpMode();
        }
    }

    public abstract void runLinearOpMode() throws InterruptedException;
    public void stopLinearOpMode() {
    }

}