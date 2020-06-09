package org.firstinspires.ftc.teamcode.MarkOne.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MarkOne.Robot.MarkOne;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 9/14/2019
 */
@TeleOp(name = "PositionTeleOp", group = "MarkOne")
public class PositionTeleOp extends MasqLinearOpMode {
    private long start, end, sum, num;
    private MarkOne robot = new MarkOne();

    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap);
        robot.initializeTeleop();

        while(!opModeIsActive()) {
            dash.create("Heading: ", robot.trackerV2.getHeading());
            dash.update();
        }

        waitForStart();

        //new Thread(robot.trackerV2).start();
        while(opModeIsActive()) {
            robot.MECH(controller1, false, true);
            dash.create("X: ",robot.trackerV2.getGlobalX());
            dash.create("Heading: ", robot.trackerV2.getHeading());
            dash.create("Y: ",robot.trackerV2.getGlobalY());
            dash.create("Raw X: ",robot.tapeMeasure.getCurrentPosition());
            dash.create("Raw YL: ",robot.intake.motor2.getCurrentPosition());
            dash.create("Raw YR: ", robot.intake.motor1.getCurrentPosition());
            start = System.currentTimeMillis();
            robot.trackerV2.updateSystem();
            end = System.currentTimeMillis();
            long delta = end - start;
            dash.create("Current Loop Time: ", delta);
            sum += delta;
            num++;
            dash.create("Average Loop Time: ", sum/num);
            dash.update();
        }
    }
}