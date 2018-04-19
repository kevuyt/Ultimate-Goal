package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqUtilities.StopCondition;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static org.firstinspires.ftc.teamcode.AutonomusWorlds.Constants.POWER_OPTIMAL;

/**
 * Created by Archish on 4/16/18.
 */
@Autonomous(name = "MasqGeneralTesterC3: Red Strafe/Stop", group = "T")
@Disabled
public class MasqGeneralTesterC3 extends MasqLinearOpMode {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.blueLineDetector.setActive();
        robot.redLineDetector.setActive();
        robot.blueLineDetector.setMargin(20);
        robot.redLineDetector.setMargin(20);
        while (!opModeIsActive()) {
            dash.create("BLUE_SENSOR_BLUE ", robot.blueLineDetector.getBlue());
            dash.create("BLUE_SENSOR_RED ", robot.blueLineDetector.getRed());
            dash.create("BLUE DETECTOR ", robot.blueLineDetector.isBlue());
            dash.create("RED_SENSOR_BLUE ", robot.redLineDetector.getBlue());
            dash.create("RED_SENSOR_RED ", robot.redLineDetector.getRed());
            dash.create("RED DETECTOR ", robot.redLineDetector.isRed());
            dash.update();
        }
        waitForStart();
        robot.yWheel.resetEncoder();
        robot.setYTarget(robot.yWheel.getPosition());
        robot.drive(48);
        robot.stop(new StopCondition() {
            public boolean stop () {
                return !robot.redLineDetector.isRed();
            }
        }, (int) robot.imu.getAbsoluteHeading(), POWER_OPTIMAL, Direction.BACKWARD);
        robot.turnAbsolute(90, Direction.RIGHT);
        robot.strafeToY(POWER_OPTIMAL);
    }
}