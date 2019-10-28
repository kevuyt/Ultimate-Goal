package org.firstinspires.ftc.teamcode.Robots.Prototype;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 9/15/2019
 */
@Autonomous(name = "ParkAutonomous", group = "Prototype")
public class ParkAutonomous extends MasqLinearOpMode {
    private PrototypeRobot robot = new PrototypeRobot();


    @Override
    public void runLinearOpMode() throws InterruptedException{
        robot.init(hardwareMap);
        robot.blockPusher.setPosition(0);
        robot.blockRotater.setPosition(0);
        robot.blockGrabber.setPosition(1);
        robot.foundationHook.setPosition(1);
        while(!opModeIsActive()) {

            dash.create("Hello ");
            dash.update();
        }

        waitForStart();
        dash.create("Park Auto");
        dash.update();
        robot.drive(37);
    }
}
