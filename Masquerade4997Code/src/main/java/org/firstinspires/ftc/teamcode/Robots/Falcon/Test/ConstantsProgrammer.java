package org.firstinspires.ftc.teamcode.Robots.Falcon.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 10/20/18.
 * Project: MasqLib
 */
@TeleOp(name = "ConstantsProgrammer", group = "NFS")
public class ConstantsProgrammer extends MasqLinearOpMode {
    private Falcon falcon = new Falcon();
    private double kp = 0.005, ki = 0, kd = 0;
    @Override
    public void runLinearOpMode() throws InterruptedException {
        falcon.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("Hello");
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            if (controller1.xOnPress()) kp += 0.001;
            if (controller1.yOnPress()) kp -= 0.001;

            if (controller1.leftTriggerOnPress()) kd += 0.0000001;
            if (controller1.rightTriggerOnPress()) kd -= 0.0000001;

            falcon.lift.setKp(kp);
            falcon.lift.setKi(ki);
            falcon.lift.setKd(kd);
            falcon.lift.DriverControl(controller1);
            falcon.rotator.DriverControl(controller2);
            falcon.MECH(controller1);
            controller1.update();
            dash.create("Kp (+X, -Y): ", kp);
            dash.create("Kd (+LT, -RT): ", kd);
            dash.update();
        }
    }
}
