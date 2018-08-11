package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robots.Creed;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@TeleOp(name = "MECH", group = "Autonomus")
public class MECH extends MasqLinearOpMode implements Constants {
    private Creed creed = new Creed();
    public void runLinearOpMode() throws InterruptedException {
        creed.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(creed.driveTrain.leftDrive.motor1.getEncoder().getModel().CPR());
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            creed.MECH(controller1);
        }
    }
}