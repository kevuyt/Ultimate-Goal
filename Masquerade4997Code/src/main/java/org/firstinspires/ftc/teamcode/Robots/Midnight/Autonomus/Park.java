package org.firstinspires.ftc.teamcode.Robots.Midnight.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.Midnight;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 2020-01-17.
 * Project: MasqLib
 */
@Autonomous(name = "Park", group = "Testbot")
@Disabled
public class Park extends MasqLinearOpMode {
    private Midnight robot = new Midnight();
    int sleeptime;
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeAutonomous();

        while(!opModeIsActive()) {
            if (controller1.yOnPress()) sleeptime++;
            if (controller1.aOnPress()) sleeptime--;
            sleeptime = Range.clip(sleeptime, 0, 20);
            dash.create("sleep", sleeptime);
            controller1.update();
            dash.update();
        }

        waitForStart();

        sleep(sleeptime);
        robot.drive(24);

        int i = 0;
        while (i < 5) {
            dance();
            i++;
        }

    }
    private void dance() {
        robot.foundationHook.hooksDown();
        sleep(1);
        robot.foundationHook.hooksUp();
        sleep(1);
    }

}
