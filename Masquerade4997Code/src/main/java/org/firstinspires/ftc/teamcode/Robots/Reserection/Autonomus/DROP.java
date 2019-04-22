package org.firstinspires.ftc.teamcode.Robots.Reserection.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Reserection.Resurrection;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "DROP", group = "T")
public class DROP extends MasqLinearOpMode {
    private Resurrection resurrection = new Resurrection();
    public void runLinearOpMode() throws InterruptedException {
        resurrection.setStartOpenCV(true);
        resurrection.mapHardware(hardwareMap);
        resurrection.initializeAutonomous();
        while (!opModeIsActive()) {
            if (!resurrection.hangBottomSwitch.getState()) resurrection.hang.setPower(0.5);
            else resurrection.hang.setBreakMode();
            dash.create("X: ", resurrection.tracker.getGlobalX());
            dash.create("Y: ", resurrection.tracker.getGlobalY());
            resurrection.tracker.updateSystem();
            dash.update();
        }
        waitForStart();
        resurrection.tracker.reset();
        resurrection.unHang();
        resurrection.drive(3, Direction.BACKWARD);
    }
}
