package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqResources.MasqHelpers.Direction;
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
    }
    public void runLinearOpMode() throws InterruptedException {
        falcon.mapHardware(hardwareMap);

        while (!opModeIsActive()) {
            dash.create("Hello");
            dash.create(falcon.imu);
            dash.update();
        }
        waitForStart();



        while (!falcon.limitTop.isPressed() && opModeIsActive()) falcon.hangSystem.setVelocity(HANG_UP);
        falcon.hangSystem.setPower(0);
        BlockPlacement blockPlacement = getBlockPlacement((int) falcon.goldAlignDetector.getXPosition(),
                falcon.goldAlignDetector.isFound());
        falcon.drive(5);

        if (blockPlacement == BlockPlacement.CENTER) {
            falcon.drive(20);
            falcon.drive(5, Direction.BACKWARD);
            falcon.turnAbsolute(90, Direction.LEFT);
            falcon.drive(45, Direction.FORWARD, 3);
            falcon.turnAbsolute(137, Direction.LEFT);
            falcon.drive(55);
            falcon.turnAbsolute(-90, Direction.LEFT);
            falcon.drive(20);
            falcon.drive(20, Direction.BACKWARD);
            falcon.turnAbsolute(-140, Direction.LEFT);
            falcon.drive(70, Direction.FORWARD, 5);
        }
        else if (blockPlacement == BlockPlacement.LEFT) {
            falcon.turnAbsolute(40, Direction.LEFT);
            falcon.drive(25);
            falcon.drive(5, Direction.BACKWARD);
            falcon.turnAbsolute(90, Direction.LEFT);
            falcon.drive(35);
            falcon.turnAbsolute(140, Direction.LEFT);
            falcon.drive(55);
            falcon.turnAbsolute(-135, Direction.LEFT);
            falcon.drive(80, Direction.FORWARD, 5);
        }
        else {
            falcon.turnAbsolute(-35, Direction.LEFT);
            falcon.drive(25);
            falcon.drive(5, Direction.BACKWARD);
            falcon.turnAbsolute(90, Direction.LEFT);
            falcon.drive(50);
            falcon.turnAbsolute(140, Direction.LEFT);
            falcon.drive(50);
            falcon.drive(80, Direction.BACKWARD, 5);
        }
        falcon.dogeForia.stop();
    }
    public BlockPlacement getBlockPlacement (int block, boolean seen) {
        if (!seen) return BlockPlacement.LEFT;
        else if (block < 200) return BlockPlacement.CENTER;
        else return BlockPlacement.RIGHT;
    }
}