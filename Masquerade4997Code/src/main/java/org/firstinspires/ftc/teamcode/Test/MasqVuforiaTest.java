package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 9/7/17.
 */
@Autonomous(name = "TEST: VUFORIA", group = "Auto")
public class MasqVuforiaTest extends MasqLinearOpMode {
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.vuforia.setPositionOne(0,500,0);
        robot.vuforia.setOrientationOne(90,0,90);
        robot.vuforia.setPositionTwo(0,500,0);
        robot.vuforia.setOrientationTwo(90,0,90);
        robot.vuforia.setPositionThree(0,500,0);
        robot.vuforia.setOrientationThree(90,0,90);
        robot.vuforia.init();
        while (!opModeIsActive()){
            dash.create("IS SEEN", robot.vuforia.isSeen("Wheels"));
            dash.create("POSITION", robot.vuforia.position("Wheels"));
        }
    }
}
