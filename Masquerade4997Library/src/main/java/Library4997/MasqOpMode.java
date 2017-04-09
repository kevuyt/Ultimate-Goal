package Library4997;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * This is a Masquerade OpMode. It includes a MasqRobot and Dashboard along with the custom gamepads.
 */

public abstract class MasqOpMode extends OpMode {
    public MasqRobot robot = new MasqRobot();
    public DashBoard dash = new DashBoard(super.telemetry);
    protected MasqController controller1 = new MasqController(super.gamepad1, "gamepad1");
    protected MasqController controller2 = new MasqController(super.gamepad2, "gamepad2");
}
