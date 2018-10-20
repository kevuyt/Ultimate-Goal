package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Falcon;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 10/20/18.
 * Project: MasqLib
 */
@TeleOp(name = "LiftEquationCreater", group = "NFS")

public class LiftEquationCreater extends MasqLinearOpMode {
    private Falcon falcon = new Falcon();
    private double rotatorKp = 0.01, rotatorBase = 0.3;
    @Override
    public void runLinearOpMode() throws InterruptedException {
        falcon.mapHardware(hardwareMap);
        falcon.lift.setClosedLoop(false);
        while (!opModeIsActive()) {
            dash.create("HELLO ");
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            if (controller1.aOnPress()) {
                rotatorKp += 0.001;
            }
            if (controller1.bOnPress()) {
                rotatorKp -= 0.001;
            }
            if (controller1.xOnPress()) {
                rotatorBase += 0.01;
            }
            if (controller1.yOnPress()) {
                rotatorBase -= 0.01;
            }

            falcon.lift.setPower(controller1.leftStickY());
            falcon.rotator.DriverControl(controller1);

            dash.create("Lift Position: ", falcon.lift.getCurrentPosition());
            dash.create("KP: ", falcon.rotator.getKP());
            dash.create("BASE: ", falcon.rotator.getBasePower());
            dash.update();
        }

    }
}
