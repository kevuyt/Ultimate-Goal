package Library4997.MasqControlSystems.MasqPurePursuit;

import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqMath.MasqPoint;

/**
 * Created by Archishmaan Peyyety on 2020-01-23.
 * Project: MasqLib
 */
public class MasqWayPointV2 implements MasqHardware {
    private double x, y, h, targetRadius = 1, modeSwitchRadius = 10, pointSwitchRadius = 10,
            minVelocity = 0.5, maxVelocity = 1, timeout = 2, lookAhead = 10, angularCorrectionSpeed = 0.015;

    private String name;
    private PointMode switchMode = PointMode.SWITCH;

    public enum PointMode {
        MECH, TANK, SWITCH
    }

    public MasqWayPointV2(){}

    public MasqWayPointV2(double x, double y, double h, double targetRadius, double modeSwitchRadius,
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
    public MasqWayPointV2(MasqPoint p, double targetRadius, double modeSwitchRadius, double pointSwitchRadius,
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

    public MasqWayPointV2 setSwitchMode(PointMode switchMode) {
        this.switchMode = switchMode;
        return this;
    }

    public MasqWayPointV2 setPoint(MasqPoint p) {
        this.x = p.getX();
        this.y = p.getY();
        this.h = p.getH();
        return this;
    }

    public MasqWayPointV2 setPoint(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
        return this;
    }

    public MasqWayPointV2 setX(double x) {
        this.x = x;
        return this;
    }

    public MasqWayPointV2 setY(double y) {
        this.y = y;
        return this;
    }

    public MasqWayPointV2 setH(double h) {
        this.h = h;
        return this;
    }

    public MasqWayPointV2 setTargetRadius(double targetRadius) {
        this.targetRadius = targetRadius;
        return this;
    }

    public MasqWayPointV2 setModeSwitchRadius(double modeSwitchRadius) {
        this.modeSwitchRadius = modeSwitchRadius;
        return this;
    }

    public MasqWayPointV2 setPointSwitchRadius(double pointSwitchRadius) {
        this.pointSwitchRadius = pointSwitchRadius;
        return this;
    }

    public MasqWayPointV2 setMinVelocity(double minVelocity) {
        this.minVelocity = minVelocity;
        return this;
    }

    public MasqWayPointV2 setMaxVelocity(double maxVelocity) {
        this.maxVelocity = maxVelocity;
        return this;
    }

    public MasqWayPointV2 setTimeout(double timeout) {
        this.timeout = timeout;
        return this;
    }

    public MasqWayPointV2 setLookAhead(double lookAhead) {
        this.lookAhead = lookAhead;
        return this;
    }

    public MasqWayPointV2 setAngularCorrectionSpeed(double angularCorrectionSpeed) {
        this.angularCorrectionSpeed = angularCorrectionSpeed;
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

    public MasqWayPointV2 setName(String name) {
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
