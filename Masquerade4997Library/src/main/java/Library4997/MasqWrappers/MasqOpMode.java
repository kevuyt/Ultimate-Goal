package Library4997.MasqWrappers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import Library4997.MasqRobot;

/**
 * This is a Masquerade OpMode. It includes a MasqRobot and Dashboard along with the custom gamepads.
 */

public abstract class MasqOpMode extends OpMode {
    public MasqRobot robot = new MasqRobot(hardwareMap);
    public DashBoard dash = new DashBoard(super.telemetry);
    protected MasqController controller1 = new MasqController (super.gamepad1, "controller1");
    protected MasqController controller2 = new MasqController (super.gamepad2, "controller2");
}
