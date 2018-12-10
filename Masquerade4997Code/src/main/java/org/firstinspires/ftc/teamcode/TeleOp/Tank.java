package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 9/15/18.
 * Project: MasqLib
 */
@TeleOp(name = "TANK", group = "Tank")
@Disabled
public class Tank extends MasqLinearOpMode implements Constants {
    private Falcon falcon = new Falcon();
    @Override
    public void runLinearOpMode() throws InterruptedException {
        falcon.mapHardware(hardwareMap);
        falcon.goldAlignDetector.disable();
        falcon.initializeTeleop();
        falcon.driveTrain.setClosedLoop(true);
        falcon.dumper.setPosition(DUMPER_IN);
        while (!opModeIsActive()) {
            dash.create("HELLO ");
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            falcon.TANK(controller1);
            if (controller1.a()) falcon.adjuster.setPosition(ADJUSTER_OUT);
            else if (controller1.y()) falcon.adjuster.setPosition(ADJUSTER_IN);

            if (controller1.leftBumper()) falcon.collector.setPower(.5);
            else if (controller1.leftTriggerPressed()) falcon.collector.setPower(-.5);
            else falcon.collector.setPower(0);


            if (controller2.b()) falcon.dumper.setPosition(DUMPER_OUT);
            else falcon.dumper.setPosition(DUMPER_IN);


            falcon.rotator.DriverControl(controller2);
            falcon.rotator.setLiftPosition(falcon.lift.getCurrentPosition());

            //Dash
            dash.create("Lift Position: ", falcon.rotator.getPosition());
            //dash.create("Rotator Goal Power: ", falcon.rotator.getBasePower());
            dash.create("Rotator Power After PID: ", falcon.rotator.getRawPower());
            dash.create("Rotator Angle: ", falcon.rotator.getAngle());
            dash.update();
        }
    }
}
