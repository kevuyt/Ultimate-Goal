package Library4997.MasqSensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import Library4997.MasqResources.MasqHelpers.MasqHardware;

import static Library4997.MasqResources.MasqUtils.formatAngle;


/**
 * Created by Archish on 1/8/18.
 */

public class MasqAdafruitIMU implements MasqHardware {
    BNO055IMU imu;
    Orientation angles;
    double zeroPos = 0;
    public MasqAdafruitIMU(String name, HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, name);
        imu.initialize(parameters);

    }
    public double getAbsoluteHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return formatAngle(angles.angleUnit, angles.firstAngle);
    }
    public double getRelativeYaw() {
        return getAbsoluteHeading() - zeroPos;
    }
    public void reset(){
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

    public String getName() {
        return "IMU";
    }
    public String[] getDash() {
        return new String[]{
                "Heading:" + Double.toString(getAbsoluteHeading()),
                "Roll" + Double.toString(getRoll()),
                "Pitch" + Double.toString(getPitch())
        };
    }
}