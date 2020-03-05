package Library4997;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqSensors.MasqAdafruitIMU;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

/**
 * Created by Archishmaan Peyyety on 8/9/18.
 * Project: MasqLib
 */

public class MasqPositionTrackerV2 implements MasqHardware, Runnable {
    private MasqMotor xSystem, yLSystem, yRSystem;
    public MasqAdafruitIMU imu;
    private double heading, globalX, globalY;
    private boolean running;

    private double prevX, prevYR, prevYL, xRadius, trackWidth, threadSleep = 1;

    public MasqPositionTrackerV2(MasqMotor xSystem, MasqMotor yLSystem, MasqMotor yRSystem, HardwareMap hardwareMap) {
        this.xSystem = xSystem;
        this.yLSystem = yLSystem;
        this.yRSystem = yRSystem;
        imu = new MasqAdafruitIMU("imu", hardwareMap);
        reset();
    }

    public double getHeading () {
        return Math.toDegrees(heading);
    }

    public void reset() {
        xSystem.resetEncoder();
        yLSystem.resetEncoder();
        yRSystem.resetEncoder();
        imu.reset();
    }

    public void updateSystem() {
        three();
    }

    private void three() {
        double xPosition = xSystem.getInches();
        double yLPosition = yLSystem.getInches();
        double yRPosition = yRSystem.getInches();
        heading = MasqUtils.adjustAngle((yLPosition - yRPosition) / trackWidth, RADIANS);
        double dX = xPosition - prevX;
        prevX = xPosition;
        double dYR = yRPosition - prevYR;
        prevYR = yRPosition;
        double dYL = yLPosition - prevYL;
        prevYL = yLPosition;
        double dH = (dYL - dYR) / trackWidth;
        double dTranslationalY = (dYR + dYL) / 2;
        double angularComponentX = xRadius * dH;
        double dTranslationalX = dX - angularComponentX;
        double dGlobalX = dTranslationalX * Math.cos(heading) - dTranslationalY * Math.sin(heading);
        double dGlobalY = dTranslationalX * Math.sin(heading) + dTranslationalY * Math.cos(heading);
        globalX += dGlobalX;
        globalY += dGlobalY;
    }

    public double getThreadSleep() {
        return threadSleep;
    }

    public void setThreadSleep(double threadSleep) {
        this.threadSleep = threadSleep;
    }

    public double getGlobalX() {
        return globalX;
    }
    public double getGlobalY() {
        return globalY;
    }

    public void setXRadius(double xRadius) {
        this.xRadius = xRadius;
    }

    public void setTrackWidth(double trackWidth) {
        this.trackWidth = trackWidth;
    }

    public void setRunning(boolean running) {
        this.running = running;
    }

    @Override
    public String getName() {
        return "Tracker";
    }

    @Override
    public String[] getDash() {
        return new String[] {
                getName() +
                "GlobalX: " + globalX,
                "GlobalY: " + globalY,
                "Heading: " + getHeading(),
        };
    }

    @Override
    public void run() {
        while (running) {
            updateSystem();
            try {
                Thread.sleep((long) threadSleep);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
