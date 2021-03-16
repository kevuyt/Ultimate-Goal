package MasqueradeLibrary.MasqSensors;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static MasqueradeLibrary.MasqResources.MasqUtils.formatAngle;
import static java.util.Locale.US;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public class MasqAdafruitIMU {
    private BNO055IMU imu;
    private Orientation angles;
    private double zeroHeading = 0, zeroPitch = 0, zeroRoll = 0;
    private String name;

    public MasqAdafruitIMU(String name, HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, name);
        imu.initialize(parameters);
        this.name = name;
    }

    public MasqAdafruitIMU(HardwareMap hardwareMap) {
        new MasqAdafruitIMU("IMU", hardwareMap);
    }

    public double getAbsoluteHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -formatAngle(angles.angleUnit, angles.firstAngle);
    }

    public double getAbsolutePitch() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return formatAngle(angles.angleUnit, angles.thirdAngle);
    }

    public double getAbsoluteRoll() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return formatAngle(angles.angleUnit, angles.secondAngle);
    }

    public double getHeading() {
        return getAbsoluteHeading() - zeroHeading;
    }

    public double getPitch() {
        return getAbsolutePitch() - zeroPitch;
    }

    public double getRoll() {
        return getAbsoluteRoll() - zeroRoll;
    }

    public void reset() {
        zeroHeading = getAbsoluteHeading();
        zeroPitch = getAbsolutePitch();
        zeroRoll = getAbsoluteRoll();
    }

    public double x() {return imu.getPosition().x;}

    public double y() {return imu.getPosition().y;}

    public double z() {return imu.getPosition().z;}

    @NonNull
    @Override
    public String toString() {
        return String.format(US, "%s:\nHeading: %.1f\nPitch: %.1f\nRoll: %.1f", name, getHeading(), getPitch(), getRoll());
    }
}