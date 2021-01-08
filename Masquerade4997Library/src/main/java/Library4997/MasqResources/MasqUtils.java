package Library4997.MasqResources;

import android.graphics.Point;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.Rect;

import java.util.Locale;

import Library4997.MasqResources.MasqMath.MasqPIDController;
import Library4997.MasqResources.MasqMath.MasqVector;
import Library4997.MasqWrappers.DashBoard;
import Library4997.MasqWrappers.MasqLinearOpMode;

import static Library4997.MasqWrappers.DashBoard.getDash;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;


/**
 * Created by Archish on 10/16/17.
 */

public class MasqUtils {
    private static MasqLinearOpMode linearOpMode;
    public static final double DEFAULT_SLEEP_TIME = 0.5;
    public static final double DEFAULT_TIMEOUT = 2;
    public static final double DEFAULT_SPEED_MULTIPLIER = Math.sqrt(2);
    public static final double DEFAULT_TURN_MULTIPLIER = 1;
    public static final double ODS_WHITE = 0.7, ODS_BLACK = 0.3;
    public static final String VUFORIA_KEY = "Ac5sAIr/////AAABmeUEovYOek9pkuVkMLDtWVGIkr+aSwnxHoPcO" +
            "Wo55EZxWMznvy9o+sR4uE8cUkHfJ2QywQNfK9SgCKSgjjRXD1lJvl3xiT0ddSjfE8JT9NMvGojoFG3nkaQP+Sq" +
            "MGTgr25mUnTM3Y7v5kcetBEF1+vIcQL28SnoWDfGGMQ9Yt9IHo/W/72s5qWMCJLSS7/8X+Scybt98htjPVAOPI" +
            "dcudmKVGUMIK5ajH8riMC/2i80n57oBV3YmEYFKq0kIl1/Yf0KP3Hre8pA2les4GgriDHZBmp/E/ixOo1H924+" +
            "DrFzuLwkk7gs7kk4Jrdp1+jqrxPBJdr8MjYjtXjW+epFt1lcvIlP/4MK44iEH9AMQXYD9";

    public static MasqPIDController turnController;
    public static MasqPIDController xySpeedController;
    public static MasqPIDController xSpeedController;
    public static MasqPIDController ySpeedController;
    public static MasqPIDController xyAngleController;
    public static MasqPIDController driveController;
    public static MasqPIDController angleController;

    public static void sleep(double timeSeconds) {
        try {
            Thread.sleep((long) timeSeconds * 1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
    public static void sleep() {sleep(DEFAULT_SLEEP_TIME);}
    public static void setLinearOpMode(MasqLinearOpMode opMode) {
        linearOpMode = opMode;
    }

    public static double adjustAngle(double angle, AngleUnit angleUnit) {
        return angleUnit.normalize(angle);
    }
    public static double adjustAngle(double angle) {
        return adjustAngle(angle, DEGREES);
    }

    public static boolean tolerance(double value1, double value2, double tolerance) {
        return Math.abs(value1 - value2) < tolerance;
    }
    public static MasqLinearOpMode getLinearOpMode () {
        return linearOpMode;
    }
    public static boolean opModeIsActive() {
        return linearOpMode.opModeIsActive();
    }
    public static HardwareMap getHardwareMap() {
        return linearOpMode.hardwareMap;
    }

    public static double max(double... vals) {
        double max = Double.MIN_VALUE;
        for (double d: vals) if (max < d) max = d;
        return max;
    }
    public static double min(double... vals) {
        double min = Double.MAX_VALUE;
        for (double d: vals) if (min > d) min = d;
        return min;
    }
    public static double scaleNumber(double m, double currentMin, double currentMax, double newMin, double newMax) {
        return (((m - currentMin) * (newMax - newMin)) / (currentMax - currentMin)) + newMin;
    }
    public static double scaleNumber(double m, double newMin, double newMax) {
        return scaleNumber(m, 0, 1, newMin, newMax);
    }
    public static Double formatAngle(AngleUnit angleUnit, double angle) {
        return Double.valueOf(formatDegrees(DEGREES.fromUnit(angleUnit, angle)));
    }
    private static String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", DEGREES.normalize(degrees));
    }
    public static Point getCenterPoint(Rect rect) {
        return new Point(rect.x + rect.width/2, rect.y + rect.height/2);
    }
    public static MasqVector getLookAhead(MasqVector initial, MasqVector current, MasqVector finalPos, double lookAhead) {
        MasqVector pathDisplacement = initial.displacement(finalPos);
        MasqVector untransformedProjection = new MasqVector(
                current.projectOnTo(pathDisplacement).getX() - initial.getX(),
                current.projectOnTo(pathDisplacement).getY() - initial.getY()).projectOnTo(pathDisplacement);
        MasqVector projection = new MasqVector(
                untransformedProjection.getX() + initial.getX(),
                untransformedProjection.getY() + initial.getY());
        double theta = Math.atan2(pathDisplacement.getY(), pathDisplacement.getX());
        return new MasqVector(
                projection.getX() + (lookAhead * cos(theta)),
                projection.getY() + (lookAhead * sin(theta)));
    }

    //https://www.desmos.com/calculator/putboztuqn
    public static MasqVector getLookAheadv2 (MasqVector initial, MasqVector current, MasqVector finalPos, double lookAhead) {
        if(finalPos.getY() == initial.getY()) return new MasqVector(current.getX(), initial.getY());
        if(finalPos.getX() == initial.getX() ) return new MasqVector(initial.getX(),current.getY());

        double slope1 = (finalPos.getY()-initial.getY())/(finalPos.getX()-initial.getX());
        double slope2 = -1/slope1;
        double theta = Math.atan2(finalPos.getY()-initial.getY(),finalPos.getX()-initial.getX());
        double x = ((slope2 * current.getX()) + initial.getY() - current.getY() - (slope1 * initial.getX()))/(slope2-slope1);

        MasqVector projection = new MasqVector(x, (x-initial.getX())*slope1 + initial.getY());

        if(finalPos.distanceToVector(initial) > (finalPos.distanceToVector(projection) - lookAhead)) {
            getDash().update();
            return new MasqVector(projection.getX() + lookAhead * cos(theta), projection.getY() + lookAhead * sin(theta));
        }
        getDash().update();
        return new MasqVector(projection.getX() - lookAhead*cos(theta), projection.getY() - lookAhead*sin(theta));
    }

}