package org.firstinspires.ftc.teamcode.Robots.MarkOne.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqPositionTracker;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 11/17/2019
 */
@TeleOp(name = "Position Getter", group = "MarkOne")
@Disabled
public class TestTeleop extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeTeleop();

        while (!opModeIsActive()) {
            dash.create("Manual Inches: ",robot.intake.motor2.getCurrentPosition() /
                    (1440 / (2 * Math.PI)));
            dash.update();
        }
        // DO NOT ADD MECH INSIDE. That will set velocity of wheels to 0
        // and then you will not be able to roll the bot to positions easier.
        while(opModeIsActive()) {
            dash.create("X: ",robot.tracker.getGlobalX());
            dash.create("Y: ",robot.tracker.getGlobalY());
            dash.create("H: ",robot.tracker.getHeading());
            dash.create("Raw X: ",robot.intake.motor1.getCurrentPosition());
            dash.create("Raw Y: ",robot.intake.motor2.getCurrentPosition());
            robot.tracker.updateSystem();
            dash.update();
        }
    }
}
