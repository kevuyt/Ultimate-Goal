package org.firstinspires.ftc.teamcode.PlaceHolder.TeleOp;

import org.firstinspires.ftc.teamcode.PlaceHolder.Robot.PlaceHolder;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 11/9/2020
 */
public class RobotTeleop extends MasqLinearOpMode {
    private PlaceHolder robot = new PlaceHolder();

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeTeleop();

        while(!opModeIsActive()) {
            dash.create("Initialized");
            dash.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            robot.MECH(controller1);
            robot.toggleClawServo(controller1.y());
            robot.intake.setPower(controller1.rightTrigger()-controller1.leftTrigger());
            if(controller1.rightBumperOnPress()) robot.shoot();
        }
    }
}
