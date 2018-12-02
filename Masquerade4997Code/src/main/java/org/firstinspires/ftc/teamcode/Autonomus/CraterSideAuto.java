package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 12/1/18.
 * Project: MasqLib
 */
@Autonomous(name = "CraterSideAuto", group = "Autonomus")
public class CraterSideAuto extends MasqLinearOpMode implements Constants {
    Falcon falcon = new Falcon();
    enum BlockPlacement {
        LEFT,
        RIGHT,
        CENTER,
        OUT_OF_BOUNDS
    }
    public void runLinearOpMode() throws InterruptedException {
        falcon.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("Hello");
            dash.update();
        }
        waitForStart();
        while (!falcon.limitTop.isPressed()) falcon.hangSystem.setVelocity(HANG_UP);
        falcon.hangSystem.setPower(0);
    }
    public BlockPlacement getBlockPlacement (int block) {
        //if ()
        return BlockPlacement.CENTER;
    }
}