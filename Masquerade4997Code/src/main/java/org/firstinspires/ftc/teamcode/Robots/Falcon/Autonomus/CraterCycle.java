package org.firstinspires.ftc.teamcode.Robots.Falcon.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Resources.BlockPlacement;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "CraterCycle", group = "T")
public class CraterCycle extends MasqLinearOpMode implements Constants {
    private Falcon falcon = new Falcon();
    private MasqPIDController liftController = new MasqPIDController(0.001, 0, 0.00001);
    private double collectExtension = -3000;
    private double scoreExtension = -2000;
    private MasqPoint rightSample = new MasqPoint(22, -20);
    private MasqPoint marker = new MasqPoint(-10, 58);
    private MasqPoint leftSample = new MasqPoint(23, 9);
    private MasqPoint centerSample = new MasqPoint(24, 0, 0);
    private MasqPoint lineup = new MasqPoint(14, 39);
    private MasqPoint collect = new MasqPoint(16, 41);
    private MasqPoint score = new MasqPoint(2, 23, -45);
    public void runLinearOpMode() throws InterruptedException {
        falcon.setStartOpenCV(false);
        falcon.mapHardware(hardwareMap);
        falcon.initializeAutonomous();
        while (!opModeIsActive()) {
            falcon.tracker.updateSystem();
            dash.create("X: ", falcon.tracker.getGlobalX());
            dash.create("Y: ", falcon.tracker.getGlobalY());
            dash.create("H: ", falcon.tracker.getHeading());
            dash.update();
        }
        waitForStart();
        falcon.tracker.reset();
        falcon.setLookAheadDistance(5);
        BlockPlacement placement = BlockPlacement.CENTER;
        if (placement == BlockPlacement.CENTER) {
            falcon.setTimeout(2);
            falcon.gotoXY(centerSample, 0, 0.8);
            falcon.setTimeout(1);
            falcon.gotoXY(14, 0, 0, 1.8);
            falcon.setTimeout(2);
            falcon.gotoXY(lineup, 30, 0.6, 0.01);
            falcon.gotoXY(marker, 45, 0.7, 0.05);
            falcon.gotoXY(0, 50, 45, 0.7);
            falcon.setTimeout(2);
            falcon.setLookAheadDistance(.5);
            falcon.gotoXY(collect, -140, 0.3, 0.07);
        }
        else if (placement == BlockPlacement.LEFT) {
            falcon.gotoXY(9, 0, 0, 0.7);
            falcon.gotoXY(leftSample, 20, 0.8);
            falcon.gotoXY(-1, 62, 45);
            falcon.gotoXY(collect, 45);
        }
        else if (placement == BlockPlacement.RIGHT) {
            falcon.gotoXY(9, 0, 0, 0.9);
            falcon.gotoXY(rightSample, 0, 0.5);
            falcon.gotoXY(12, -16, 0, 0.5);
            falcon.gotoXY(lineup, 20, 0.9, 0.007);
            falcon.gotoXY(-7, 58, 45);
            falcon.gotoXY(collect, 45);
        }
        initalDump();
    }
    public void initalDump() {
        extendLift(collectExtension, -1);
        sleep(1);
        extendLift(scoreExtension, 1);
        runSimultaneously(new Runnable() {
            @Override
            public void run() {
                while (!falcon.rotateTopSwitch.isPressed()) falcon.rotator.rotator.setPower(-1);
            }
        }, new Runnable() {
            @Override
            public void run() {
                falcon.gotoXY(score, -45, 1.2);
            }
        });
    }
    public void extendLift(double position, double initalVelocity) {
        falcon.lift.lift.setVelocity(initalVelocity);
        MasqClock clock = new MasqClock();
        while (!clock.elapsedTime(2, MasqClock.Resolution.SECONDS) || Math.abs(falcon.lift.lift.getVelocity()) < 0.1)
            falcon.lift.lift.setVelocity(liftController.getOutput(falcon.lift.lift.getCurrentPosition(), position));

    }
}
