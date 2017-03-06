package team4997lib;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import BasicLib4997.Controller;
import BasicLib4997.DashBoard;
import BasicLib4997.MasqMotors.MasqRobot.MasqRobot;

/**
 * This is a Masquerade OpMode. It includes a MasqRobot and MasqTelemetry.
 */

public abstract class MasqOpMode extends OpMode {

    public MasqRobot robot = new MasqRobot();
    public DashBoard dash = new DashBoard(super.telemetry);
    protected Controller controller1 = new Controller(super.gamepad1);
    protected Controller controller2 = new Controller(super.gamepad2);
}
