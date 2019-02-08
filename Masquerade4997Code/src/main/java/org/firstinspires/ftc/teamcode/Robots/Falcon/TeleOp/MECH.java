package org.firstinspires.ftc.teamcode.Robots.Falcon.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 10/27/18.
 * Project: MasqLib
 */
@TeleOp(name = "MECH", group = "NFS")
public class MECH extends MasqLinearOpMode implements Constants {
    private Falcon falcon = new Falcon();
    @Override
    public void runLinearOpMode()  {
        falcon.setStartOpenCV(false);
        falcon.mapHardware(hardwareMap);
        falcon.initializeTeleop();
        falcon.hang.setClosedLoop(true);
        falcon.driveTrain.setClosedLoop(true);
        falcon.dumper.setPosition(DUMPER_IN);
        while (!opModeIsActive()) {
            dash.create("HELLO ");
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            falcon.MECH(controller1);
            if (controller1.leftBumper()) falcon.collector.setPower(.5);
            else if (controller1.leftTriggerPressed()) falcon.collector.setPower(-.5);
            else falcon.collector.setPower(0);

            if (controller2.b()) falcon.dumper.setPosition(DUMPER_OUT);
            else falcon.dumper.setPosition(DUMPER_IN);

            if (controller2.leftStickY() < 0 && !falcon.limitTop.isPressed()) falcon.hang.setPower(-1);
            else if (controller2.leftStickY() > 0 && !falcon.limitBottom.isPressed()) falcon.hang.setPower(1);
            else falcon.hang.setPower(0);

            falcon.rotator.DriverControl(controller2);
            falcon.rotator.setLiftPosition(falcon.lift.getCurrentPosition());
            falcon.lift.DriverControl(controller1);
            dash.create("Top: ", falcon.limitTop.isPressed());
            dash.create("Left: ", falcon.limitBottom.isPressed());
            dash.update();
            controller1.update();
        }
    }
}
