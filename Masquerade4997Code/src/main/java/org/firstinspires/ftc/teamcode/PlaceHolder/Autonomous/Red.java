package org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous.Vision.HeightFinder;
import org.firstinspires.ftc.teamcode.PlaceHolder.Robot.PlaceHolder;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous.Vision.HeightFinder.TargetZone.A;
import static org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous.Vision.HeightFinder.TargetZone.B;

/**
 * Created by Keval Kataria on 9/12/2020
 */
@Autonomous(name = "Red", group = "PlaceHolder")
public class Red extends MasqLinearOpMode {
    private PlaceHolder robot = new PlaceHolder();
    private HeightFinder.TargetZone zone;

    private MasqWayPoint zoneA = new MasqWayPoint().setMaxVelocity(1).setMinVelocity(0.7)
            .setOnComplete(() ->{robot.claw.setPosition(1);}).setPoint(-11.5, -111.5,0).setTimeout(5);
    private MasqWayPoint zoneB = new MasqWayPoint().setMaxVelocity(1).setMinVelocity(0.7)
            .setOnComplete(() ->{robot.claw.setPosition(1);}).setPoint(11.5, -106,0).setTimeout(4);
    private MasqWayPoint zoneC = new MasqWayPoint().setMaxVelocity(1).setMinVelocity(0.7)
            .setOnComplete(() ->{robot.claw.setPosition(1);}).setPoint(-11.5, -94,0).setTimeout(4);

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeAutonomous();
        robot.initCamera(hardwareMap);

        while(!opModeIsActive()) {
            zone = HeightFinder.findZone(robot.camera.detector);
            dash.create("Zone: " + zone);
            dash.update();
        }

        waitForStart();

        timeoutClock.reset();

        MasqWayPoint target;
        if (zone == A) target = zoneA;
        else if (zone == B) target = zoneB;
        else target = zoneC;

        robot.xyPath(5, target);
        robot.xyPath(2, new MasqWayPoint().setPoint(7.5, -65, 180));
        robot.shoot();
        robot.xyPath(2, new MasqWayPoint().setPoint(14.5, -65, 180));
        robot.shoot();
        robot.xyPath(2, new MasqWayPoint().setPoint(23, -65, 180));
        robot.shoot();
        robot.xyPath(2, new MasqWayPoint().setPoint(23,-80, 180));
    }
}
