package BasicLib4997;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import BasicLib4997.MasqMotors.TankDrive.MasqRobot;

/**
 * Custom Linear opMode
 */

public abstract class MasqLinearOpMode extends LinearOpMode {

    protected DashBoard dashBoard;
    public MasqRobot robot;
    public final void runOpMode() throws InterruptedException {
        try {
            dashBoard = new DashBoard(super.telemetry);
            runLinearOpMode();
        } finally {
            stopLinearOpMode();
        }
    }

    public abstract void runLinearOpMode() throws InterruptedException;
    public void stopLinearOpMode() {
    }

}