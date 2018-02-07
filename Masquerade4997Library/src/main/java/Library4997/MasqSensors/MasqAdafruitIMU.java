package Library4997.MasqSensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import Library4997.MasqUtilities.MasqHardware;


/**
 * Created by Archish on 10/28/16.
 */

public class MasqAdafruitIMU implements MasqHardware{
    private final BNO055IMU imu;
    private final String name;
    private double zeroPos;
    public MasqAdafruitIMU(String name, HardwareMap hardwareMap) {
        this.name = name;
        imu = hardwareMap.get(BNO055IMU.class, name);
        setParameters();
    }

    private void setParameters() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.useExternalCrystal = true;
        parameters.angleUnit = com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit.RADIANS;
        parameters.pitchMode = com.qualcomm.hardware.bosch.BNO055IMU.PitchMode.WINDOWS;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
    }
    private double[] getAngles() {
        Quaternion quatAngles = imu.getQuaternionOrientation();
        double w = quatAngles.w;
        double x = quatAngles.x;
        double y = quatAngles.y;
        double z = quatAngles.z;
        double roll = Math.atan2( 2*(w*x + y*z) , 1 - 2*(x*x + y*y) ) * 180.0 / Math.PI;
        double pitch = Math.asin( 2*(w*y - x*z) ) * 180.0 / Math.PI;
        double yaw = Math.atan2( 2*(w*z + x*y), 1 - 2*(y*y + z*z) ) * 180.0 / Math.PI;
        return new double[]{yaw, pitch, roll};
    }
    public double adjustAngle(double angle) {
        while (angle > 180)  angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
    public double getHeading() {
        return getAngles()[0];
    }
    public double getYaw () {
        return getHeading() - zeroPos;
    }
    public void reset(){
        zeroPos = getHeading();
    }
    public double getPitch() {
        return getAngles()[1];
    }
    public double getRoll() {
        return getAngles()[2];
    }
    public String getName() {
        return name;
    }
    public String[] getDash() {
        return new String[]{
                "Heading:" + Double.toString(getHeading()),
                "Roll" + Double.toString(getRoll()),
                "Pitch" + Double.toString(getPitch()),
        };
    }
}
