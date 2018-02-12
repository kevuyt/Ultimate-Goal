package SubSystems4997.MasqWrappers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import Library4997.MasqUtilities.MasqAnnotations.MasqOpModeClass;
import Library4997.MasqWrappers.*;
import Library4997.MasqWrappers.MasqController;

/**
 * This is a Masquerade OpMode. It includes a MasqRobot and Dashboard along with the custom gamepads.
 */
@MasqOpModeClass
public abstract class MasqOpMode extends OpMode {
    public MasqRobot robot = new MasqRobot();
    public Library4997.MasqWrappers.DashBoard dash = new Library4997.MasqWrappers.DashBoard(super.telemetry);
    protected Library4997.MasqWrappers.MasqController controller1 = new Library4997.MasqWrappers.MasqController(super.gamepad1, "controller1");
    protected Library4997.MasqWrappers.MasqController controller2 = new MasqController(super.gamepad2, "controller2");
}
