package MasqLibrary.MasqSensors;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.*;

import org.firstinspires.ftc.robotcore.external.navigation.*;

import static MasqLibrary.MasqResources.MasqUtils.*;
import static com.qualcomm.hardware.bosch.BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
import static com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit.DEGREES;
import static com.qualcomm.hardware.bosch.BNO055IMU.SensorMode.IMU;
import static java.util.Locale.US;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public class MasqIMU {
    private BNO055IMU imu;
    private Orientation angles;
    private double zeroHeading, zeroPitch, zeroRoll;
    private String name;

    public MasqIMU(String name) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = IMU;
        parameters.angleUnit = DEGREES;
        parameters.accelUnit = METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = getHardwareMap().get(BNO055IMU.class, name);
        imu.initialize(parameters);
        this.name = name;
    }
    public MasqIMU() {this("imu");}

    public double getAbsoluteHeading() {
        angles = imu.getAngularOrientation(INTRINSIC, ZYX, AngleUnit.DEGREES);
        return -formatAngle(angles.angleUnit, angles.firstAngle);
    }
    public double getAbsolutePitch() {
        angles = imu.getAngularOrientation(INTRINSIC, ZYX, AngleUnit.DEGREES);
        return formatAngle(angles.angleUnit, angles.thirdAngle);
    }
    public double getAbsoluteRoll() {
        angles = imu.getAngularOrientation(INTRINSIC, ZYX, AngleUnit.DEGREES);
        return formatAngle(angles.angleUnit, angles.secondAngle);
    }
    public double getHeading() {return getAbsoluteHeading() - zeroHeading;}
    public double getPitch() {return getAbsolutePitch() - zeroPitch;}
    public double getRoll() {return getAbsoluteRoll() - zeroRoll;}

    public double x() {return imu.getPosition().x;}
    public double y() {return imu.getPosition().y;}
    public double z() {return imu.getPosition().z;}

    public void reset() {
        zeroHeading = getAbsoluteHeading();
        zeroPitch = getAbsolutePitch();
        zeroRoll = getAbsoluteRoll();
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(US, "%s:\nHeading: %.1f\nPitch: %.1f\nRoll: %.1f", name,
                getHeading(), getPitch(), getRoll());
    }
}