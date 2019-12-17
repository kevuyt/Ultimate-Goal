package org.firstinspires.ftc.teamcode.Robots.MarkOne.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;
import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.CVInterpreter;

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

        while(!opModeIsActive()) {
            dash.create("Position: ", CVInterpreter.getPosition(robot.cv.detector));
            dash.update();
        }
        waitForStart();

        robot.cv.stop();
    }
}
