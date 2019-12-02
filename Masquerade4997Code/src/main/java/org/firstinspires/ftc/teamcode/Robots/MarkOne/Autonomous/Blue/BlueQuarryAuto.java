package org.firstinspires.ftc.teamcode.Robots.MarkOne.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;
import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOneDetector;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 12/1/2019
 */
@Autonomous(name = "BlueQuarry", group = "MarkOne")
public class BlueQuarryAuto extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeAutonomous();

        MarkOneDetector.SkystonePosition position = robot.detector.getPosition();

        while(!opModeIsActive()) {
            dash.create("Position: ", position);
            dash.update();
        }
        waitForStart();

        robot.foundationHook.mid();
        robot.blockPusher.setPosition(1);
        sleep();

        if(position == MarkOneDetector.SkystonePosition.LEFT) robot.runStoneLeft();
        else if (position == MarkOneDetector.SkystonePosition.MIDDLE) robot.runStoneMiddle();
        else if (position == MarkOneDetector.SkystonePosition.RIGHT) robot.runStoneRight();

        robot.detector.stop();
    }
}
