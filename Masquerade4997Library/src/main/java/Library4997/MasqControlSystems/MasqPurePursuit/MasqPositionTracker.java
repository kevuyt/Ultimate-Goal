package Library4997.MasqControlSystems.MasqPurePursuit;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.MasqHardware;

/**
 * Created by Archishmaan Peyyety on 8/9/18.
 * Project: MasqLib
 */

public class MasqPositionTracker implements MasqHardware {
    private MasqMotor xSystem, ySystem;
    public BNO055IMU imu;
    private double globalX = 0, globalY = 0, prevX = 0, prevY = 0;
    private Orientation angles;
    private double zeroPos = 0;

    public MasqPositionTracker(MasqMotor xSystem, MasqMotor ySystem, BNO055IMU imu) {
        this.imu = imu;
        this.xSystem = xSystem;
        this.ySystem = ySystem;
        reset();
    }
    public double getHeading () {
        return getRelativeYaw();
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
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return formatAngle(angles.angleUnit, angles.firstAngle);
    }
    public double getRelativeYaw() {
        return getAbsoluteHeading() - zeroPos;
    }
    public void resetIMU(){
        zeroPos = getAbsoluteHeading();
    }
    public double getPitch() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return formatAngle(angles.angleUnit, angles.thirdAngle);
    }
    public double getRoll() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return formatAngle(angles.angleUnit, angles.secondAngle);
    }

    public double x () {
        return imu.getPosition().x;
    }
    public double y () {
        return imu.getPosition().y;
    }
    public double z () {
        return imu.getPosition().z;
    }

    Double formatAngle(AngleUnit angleUnit, double angle) {
        return Double.valueOf(formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle)));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    @Override
    public String getName() {
        return "Tracker";
    }

    @Override
    public String[] getDash() {
        return new String[] {

        };
    }
}
