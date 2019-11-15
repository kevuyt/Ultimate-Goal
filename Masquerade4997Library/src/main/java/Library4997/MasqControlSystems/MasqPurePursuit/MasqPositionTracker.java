package Library4997.MasqControlSystems.MasqPurePursuit;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqSensors.MasqAdafruitIMU;

/**
 * Created by Archishmaan Peyyety on 8/9/18.
 * Project: MasqLib
 */

public class MasqPositionTracker implements MasqHardware {
    private MasqMotor xSystem, ySystem;
    public MasqAdafruitIMU imu;
    private double globalX = 0, globalY = 0, prevX = 0, prevY = 0;
    private Orientation angles;
    private double zeroPos = 0;
    private String imuName;

    public MasqPositionTracker(MasqMotor xSystem, MasqMotor ySystem, HardwareMap hardwareMap) {
        this.xSystem = xSystem;
        this.ySystem = ySystem;
        imu = new MasqAdafruitIMU("imu", hardwareMap);
        reset();
    }
    public MasqPositionTracker(MasqMotor xSystem, MasqMotor ySystem, String imuName, HardwareMap hardwareMap) {
        this.xSystem = xSystem;
        this.ySystem = ySystem;
        this.imuName = imuName;
        imu = new MasqAdafruitIMU(imuName, hardwareMap);
        reset();
    }
    public double getHeading () {
        return imu.getRelativeYaw();
    }
    public void updateSystem () {
        double deltaX = (-xSystem.getCurrentPosition() - prevX);
        double deltaY = (-ySystem.getCurrentPosition() - prevY);
        double heading = Math.toRadians(getRelativeYaw());
        double x = deltaX * Math.cos(heading) - deltaY * Math.sin(heading);
        double y = deltaX * Math.sin(heading) + deltaY * Math.cos(heading);
        globalX += x;
        globalY += y;
        prevY = -ySystem.getCurrentPosition();
        prevX = -xSystem.getCurrentPosition();
    }

    public void reset() {
        xSystem.resetEncoder();
        ySystem.resetEncoder();
        resetIMU();
    }

    public double getGlobalX() {
        return globalX * ((2 * Math.PI) / 1440);
    }
    public double getGlobalY() {
        return globalY * ((2 * Math.PI) / 1440);
    }

    public double getAbsoluteHeading() {
        return imu.getAbsoluteHeading();
    }
    public double getRelativeYaw() {
        return imu.getRelativeYaw();
    }
    public void resetIMU(){
        imu.reset();
    }
    public double getPitch() {
        return imu.getPitch();
    }
    public double getRoll() {
        return imu.getRoll();
    }

    public double x () {
        return imu.x();
    }
    public double y () {
        return imu.y();
    }
    public double z () {
        return imu.z();
    }


    @Override
    public String getName() {
        return "Tracker";
    }

    @Override
    public String[] getDash() {
        return new String[] {
            getName() +
            "Globalx: " + globalX +
            "Globaly: " + globalY
        };
    }
}
