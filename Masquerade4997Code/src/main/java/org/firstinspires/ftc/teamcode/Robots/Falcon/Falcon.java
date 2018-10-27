package org.firstinspires.ftc.teamcode.Robots.Falcon;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robots.Falcon.FalconSubSystems.MasqRotator;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqPositionTracker;
import Library4997.MasqDriveTrains.MasqDriveTrain;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqRobot;
import Library4997.MasqSensors.MasqAdafruitIMU;
import Library4997.MasqServos.MasqCRServo;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqWrappers.DashBoard;

/**
 * Created by Archishmaan Peyyety on 6/2/18.
 * Project: MasqLib
 */

public class Falcon extends MasqRobot {
    public MasqAdafruitIMU imu;
    public MasqRotator rotator;
    public MasqMotor lift;
    public MasqServo hangLatch;
    public MasqServo dumper;
    public MasqCRServo collector;
    public MasqServo adjuster;
    public MasqServo endHang;
    public MasqMotor endSpool;
    public GoldAlignDetector goldAlignDetector;
    public void mapHardware(HardwareMap hardwareMap) {
        dash = DashBoard.getDash();
        imu = new MasqAdafruitIMU("imu", hardwareMap);
        driveTrain = new MasqDriveTrain(hardwareMap);
        tracker = new MasqPositionTracker(driveTrain.leftDrive, driveTrain.rightDrive, imu);
        rotator = new MasqRotator(hardwareMap);
        lift = new MasqMotor("lift", MasqMotorModel.NEVEREST60, DcMotor.Direction.REVERSE, hardwareMap);
        hangLatch = new MasqServo("hangLatch", hardwareMap);
        dumper = new MasqServo("dumper", hardwareMap);
        collector = new MasqCRServo("collector", hardwareMap);
        adjuster = new MasqServo("adjuster", hardwareMap);
        endHang = new MasqServo("endHang", hardwareMap);
        endSpool = new MasqMotor("endSpool", MasqMotorModel.NEVEREST60, hardwareMap);
        goldAlignDetector = new GoldAlignDetector();
        startOpenCV(hardwareMap);
    }
    private void startOpenCV (HardwareMap hardwareMap) {
        goldAlignDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        goldAlignDetector.useDefaults();
        // Optional Tuning
        goldAlignDetector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        goldAlignDetector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        goldAlignDetector.downscale = 0.4; // How much to downscale the input frames
        goldAlignDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //falcon.goldAlignDetector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        goldAlignDetector.maxAreaScorer.weight = 0.005;
        goldAlignDetector.ratioScorer.weight = 5;
        goldAlignDetector.ratioScorer.perfectRatio = 1.0;
        goldAlignDetector.enable();
    }
    public void turnTillGold (double speed, Direction direction) {
        while (opModeIsActive() && !goldAlignDetector.getAligned()) {
            driveTrain.setPower(-speed * direction.value, speed * direction.value);
        }
        driveTrain.setPower(0, 0);
    }
    public void update () {
        dash.update();
        tracker.updateSystemV2();
    }
}
