package Library4997.MasqResources;

/**
 * Created by Archishmaan Peyyety on 12/14/18.
 * Project: MasqLib
 */
public class MasqPIDPackage {
    private double kpTurn, kiTurn, kdTurn,
                   kpDriveAngular, kiDriveAngular, kdDriveAngular,
                   kpDriveEncoder, kiDriveEncoder, kdDriveEncoder,
                   kpMotorTeleOp, kiMotorTeleOp, kdMotorTelOp,
                   kpMotorAuto, kiMotorAuto, kdMotorAuto;

    public MasqPIDPackage(double kpTurn, double kiTurn, double kdTurn,
                          double kpDriveAngular, double kiDriveAngular, double kdDriveAngular,
                          double kpDriveEncoder, double kiDriveEncoder, double kdDriveEncoder,
                          double kpMotorTeleOp, double kiMotorTeleOp, double kdMotorTelOp,
                          double kpMotorAuto, double kiMotorAuto, double kdMotorAuto) {
        this.kpTurn = kpTurn;
        this.kiTurn = kiTurn;
        this.kdTurn = kdTurn;
        this.kpDriveAngular = kpDriveAngular;
        this.kiDriveAngular = kiDriveAngular;
        this.kdDriveAngular = kdDriveAngular;
        this.kpDriveEncoder = kpDriveEncoder;
        this.kiDriveEncoder = kiDriveEncoder;
        this.kdDriveEncoder = kdDriveEncoder;
        this.kpMotorTeleOp = kpMotorTeleOp;
        this.kiMotorTeleOp = kiMotorTeleOp;
        this.kdMotorTelOp = kdMotorTelOp;
        this.kpMotorAuto = kpMotorAuto;
        this.kiMotorAuto = kiMotorAuto;
        this.kdMotorAuto = kdMotorAuto;
    }
    public MasqPIDPackage () {
        this.kpTurn = MasqUtils.KP.TURN;
        this.kiTurn = MasqUtils.KI.TURN;
        this.kdTurn = MasqUtils.KD.TURN;
        this.kpDriveAngular = MasqUtils.KP.DRIVE_ANGULAR;
        this.kiDriveAngular = MasqUtils.KI.DRIVE;
        this.kdDriveAngular = MasqUtils.KD.DRIVE;
        this.kpDriveEncoder = MasqUtils.KP.DRIVE_ENCODER;
        this.kiDriveEncoder = 0;
        this.kdDriveEncoder = 0;
        this.kpMotorTeleOp = MasqUtils.KP.MOTOR_TELEOP;
        this.kiMotorTeleOp = MasqUtils.KI.MOTOR_TELEOP;
        this.kdMotorTelOp = MasqUtils.KD.MOTOR_TELEOP;
        this.kpMotorAuto = MasqUtils.KP.MOTOR_AUTONOMOUS;
        this.kiMotorAuto = MasqUtils.KI.MOTOR_AUTONOMOUS;
        this.kdMotorAuto = MasqUtils.KD.MOTOR_AUTONOMOUS;
    }

    public double getKpTurn() {
        return kpTurn;
    }

    public void setKpTurn(double kpTurn) {
        this.kpTurn = kpTurn;
    }

    public double getKiTurn() {
        return kiTurn;
    }

    public void setKiTurn(double kiTurn) {
        this.kiTurn = kiTurn;
    }

    public double getKdTurn() {
        return kdTurn;
    }

    public void setKdTurn(double kdTurn) {
        this.kdTurn = kdTurn;
    }

    public double getKpDriveAngular() {
        return kpDriveAngular;
    }

    public void setKpDriveAngular(double kpDriveAngular) {
        this.kpDriveAngular = kpDriveAngular;
    }

    public double getKiDriveAngular() {
        return kiDriveAngular;
    }

    public void setKiDriveAngular(double kiDriveAngular) {
        this.kiDriveAngular = kiDriveAngular;
    }

    public double getKdDriveAngular() {
        return kdDriveAngular;
    }

    public void setKdDriveAngular(double kdDriveAngular) {
        this.kdDriveAngular = kdDriveAngular;
    }

    public double getKpDriveEncoder() {
        return kpDriveEncoder;
    }

    public void setKpDriveEncoder(double kpDriveEncoder) {
        this.kpDriveEncoder = kpDriveEncoder;
    }

    public double getKiDriveEncoder() {
        return kiDriveEncoder;
    }

    public void setKiDriveEncoder(double kiDriveEncoder) {
        this.kiDriveEncoder = kiDriveEncoder;
    }

    public double getKdDriveEncoder() {
        return kdDriveEncoder;
    }

    public void setKdDriveEncoder(double kdDriveEncoder) {
        this.kdDriveEncoder = kdDriveEncoder;
    }

    public double getKpMotorTeleOp() {
        return kpMotorTeleOp;
    }

    public void setKpMotorTeleOp(double kpMotorTeleOp) {
        this.kpMotorTeleOp = kpMotorTeleOp;
    }

    public double getKiMotorTeleOp() {
        return kiMotorTeleOp;
    }

    public void setKiMotorTeleOp(double kiMotorTeleOp) {
        this.kiMotorTeleOp = kiMotorTeleOp;
    }

    public double getKdMotorTelOp() {
        return kdMotorTelOp;
    }

    public void setKdMotorTelOp(double kdMotorTelOp) {
        this.kdMotorTelOp = kdMotorTelOp;
    }

    public double getKpMotorAuto() {
        return kpMotorAuto;
    }

    public void setKpMotorAuto(double kpMotorAuto) {
        this.kpMotorAuto = kpMotorAuto;
    }

    public double getKiMotorAuto() {
        return kiMotorAuto;
    }

    public void setKiMotorAuto(double kiMotorAuto) {
        this.kiMotorAuto = kiMotorAuto;
    }

    public double getKdMotorAuto() {
        return kdMotorAuto;
    }

    public void setKdMotorAuto(double kdMotorAuto) {
        this.kdMotorAuto = kdMotorAuto;
    }
}
