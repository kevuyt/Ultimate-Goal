package Library4997.MasqSensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import Library4997.MasqUtilities.MasqHardware;


/**
 * Created by Archish on 1/8/18.
 */

public class MasqAdafruitIMU implements MasqHardware {
    BNO055IMU imu;
    Orientation angles;
    double zeroPos = 0;
    public MasqAdafruitIMU(String name, HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, name);
        imu.initialize(parameters);

    }
    public double adjustAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
    public double getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return formatAngle(angles.angleUnit, angles.firstAngle);
    }
    public double getYaw () {
        return getHeading() - zeroPos;
    }
    public void reset(){
        zeroPos = getHeading();
    }
    public double getPitch() {
        return formatAngle(angles.angleUnit, angles.thirdAngle);
    }
    public double getRoll() {
        return formatAngle(angles.angleUnit, angles.secondAngle);
    }
    Double formatAngle(AngleUnit angleUnit, double angle) {
        return Double.valueOf(formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle)));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public String getName() {
        return "IMU";
    }
    public String[] getDash() {
        return new String[]{
                "Heading:" + Double.toString(getHeading()),
                "Roll" + Double.toString(getRoll()),
                "Pitch" + Double.toString(getPitch()),
        };
    }
}
