package Library4997;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import Library4997.MasqRobot.MasqRobot;

/**
 * This is a Masquerade OpMode. It includes a MasqRobot and Dashboard along with the custom gamepads.
 */

public abstract class MasqOpMode extends OpMode {
    public MasqRobot robot = new MasqRobot();
    public DashBoard dash = new DashBoard(super.telemetry);
    protected Controller controller1 = new Controller(super.gamepad1, "gamepad1");
    protected Controller controller2 = new Controller(super.gamepad2, "gamepad2");
}
