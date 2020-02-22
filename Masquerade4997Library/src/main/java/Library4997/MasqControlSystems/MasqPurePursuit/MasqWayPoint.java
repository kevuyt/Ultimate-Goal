package Library4997.MasqControlSystems.MasqPurePursuit;

import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqResources.MasqMath.MasqVector;

/**
 * Created by Archishmaan Peyyety on 2020-01-23.
 * Project: MasqLib
 */
public class MasqWayPoint implements MasqHardware {
    private double x, y, h, targetRadius = 1, modeSwitchRadius = 10, pointSwitchRadius = 10,
            minVelocity = 0.5, maxVelocity = 1, timeout = 2, lookAhead = 10,
            angularCorrectionSpeed = 0.02, speedBias = 0.5, driveCorrectionSpeed = 0.07;

    private String name;
    private PointMode switchMode = PointMode.SWITCH;

    private Runnable onComplete = null;

    private Runnable onSimul = null;

    public enum PointMode {
        MECH, TANK, SWITCH
    }

    public MasqWayPoint(){}

    public MasqWayPoint(double x, double y, double h, double targetRadius, double modeSwitchRadius,
                        double pointSwitchRadius, double minVelocity, double maxVelocity, double timeout,
                        double lookAhead, double angularCorrectionSpeed) {
        this.x = x;
        this.y = y;
        this.h = h;
        this.targetRadius = targetRadius;
        this.modeSwitchRadius = modeSwitchRadius;
        this.pointSwitchRadius = pointSwitchRadius;
        this.minVelocity = minVelocity;
        this.maxVelocity = maxVelocity;
        this.timeout = timeout;
        this.lookAhead = lookAhead;
        this.angularCorrectionSpeed = angularCorrectionSpeed;
    }
    public MasqWayPoint(MasqPoint p, double targetRadius, double modeSwitchRadius, double pointSwitchRadius,
                        double minVelocity, double maxVelocity, double timeout, double lookAhead, double angularCorrectionSpeed) {
        this.x = p.getX();
        this.y = p.getY();
        this.h = p.getH();
        this.targetRadius = targetRadius;
        this.modeSwitchRadius = modeSwitchRadius;
        this.pointSwitchRadius = pointSwitchRadius;
        this.minVelocity = minVelocity;
        this.maxVelocity = maxVelocity;
        this.timeout = timeout;
        this.lookAhead = lookAhead;
        this.angularCorrectionSpeed = angularCorrectionSpeed;
    }

    public MasqWayPoint setSwitchMode(PointMode switchMode) {
        this.switchMode = switchMode;
        return this;
    }

    public MasqWayPoint setPoint(MasqPoint p) {
        this.x = p.getX();
        this.y = p.getY();
        this.h = p.getH();
        return this;
    }

    public MasqWayPoint setPoint(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
        return this;
    }

    public MasqWayPoint setX(double x) {
        this.x = x;
        return this;
    }

    public MasqWayPoint setY(double y) {
        this.y = y;
        return this;
    }

    public MasqWayPoint setH(double h) {
        this.h = h;
        return this;
    }

    public MasqWayPoint setTargetRadius(double targetRadius) {
        this.targetRadius = targetRadius;
        return this;
    }

    public MasqWayPoint setModeSwitchRadius(double modeSwitchRadius) {
        this.modeSwitchRadius = modeSwitchRadius;
        return this;
    }

    public MasqWayPoint setPointSwitchRadius(double pointSwitchRadius) {
        this.pointSwitchRadius = pointSwitchRadius;
        return this;
    }

    public MasqWayPoint setMinVelocity(double minVelocity) {
        this.minVelocity = minVelocity;
        return this;
    }

    public MasqWayPoint setDriveCorrectionSpeed(double driveCorrectionSpeed) {
        this.driveCorrectionSpeed = driveCorrectionSpeed;
        return this;
    }

    public MasqWayPoint setMaxVelocity(double maxVelocity) {
        this.maxVelocity = maxVelocity;
        return this;
    }

    public MasqWayPoint setTimeout(double timeout) {
        this.timeout = timeout;
        return this;
    }

    public MasqWayPoint setLookAhead(double lookAhead) {
        this.lookAhead = lookAhead;
        return this;
    }

    public MasqWayPoint setAngularCorrectionSpeed(double angularCorrectionSpeed) {
        this.angularCorrectionSpeed = angularCorrectionSpeed;
        return this;
    }

    public MasqWayPoint setOnComplete(Runnable onComplete) {
        this.onComplete = onComplete;
        return this;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getH() {
        return h;
    }

    public double getTargetRadius() {
        return targetRadius;
    }

    public double getModeSwitchRadius() {
        return modeSwitchRadius;
    }

    public double getPointSwitchRadius() {
        return pointSwitchRadius;
    }

    public double getMinVelocity() {
        return minVelocity;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public double getTimeout() {
        return timeout;
    }

    public double getLookAhead() {
        return lookAhead;
    }

    public double getAngularCorrectionSpeed() {
        return angularCorrectionSpeed;
    }

    public PointMode getSwitchMode() {
        return switchMode;
    }

    public double getDriveCorrectionSpeed() {
        return driveCorrectionSpeed;
    }

    public MasqWayPoint setSpeedBias(double speedBias) {
        this.speedBias = speedBias;
        return this;
    }

    public double getSpeedBias() {
        return speedBias;
    }

    public Runnable getOnComplete() {
        return onComplete;
    }

    public Runnable getOnSimul() {
        return onSimul;
    }

    public MasqWayPoint setOnSimul(Runnable onSimul) {
        this.onSimul = onSimul;
        return this;
    }

    public MasqVector getPoint() {
        return new MasqVector(x,y);
    }

    public MasqWayPoint setName(String name) {
        this.name = name;
        return this;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public String[] getDash() {
        return new String[0];
    }
}