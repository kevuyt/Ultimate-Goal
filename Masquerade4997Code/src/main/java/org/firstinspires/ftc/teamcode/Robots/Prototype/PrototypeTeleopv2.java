package org.firstinspires.ftc.teamcode.Robots.Prototype;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 9/14/2019
 */
@TeleOp(name = "PrototypeTeleopv2", group = "Prototype")
public class PrototypeTeleopv2 extends MasqLinearOpMode {
    private PrototypeRobot robot = new PrototypeRobot();

    @Override
    public void runLinearOpMode() {
        robot.mapHardware(hardwareMap);
        robot.driveTrain.setClosedLoop(true);
        while(!opModeIsActive()) {
            dash.create("Big Brain Time");
            dash.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            robot.MECH(controller1);

        }
    }
}
