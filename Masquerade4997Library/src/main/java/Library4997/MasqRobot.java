package Library4997;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPointLegacy;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqResources.MasqMath.MasqVector;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqWrappers.DashBoard;
import Library4997.MasqWrappers.MasqController;
import Library4997.MasqWrappers.MasqPredicate;

import static Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint.PointMode.MECH;
import static Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint.PointMode.SWITCH;
import static Library4997.MasqResources.MasqUtils.DEFAULT_SLEEP_TIME;
import static Library4997.MasqResources.MasqUtils.DEFAULT_TIMEOUT;
import static Library4997.MasqResources.MasqUtils.angleController;
import static Library4997.MasqResources.MasqUtils.driveController;
import static Library4997.MasqResources.MasqUtils.scaleNumber;
import static Library4997.MasqResources.MasqUtils.turnController;
import static Library4997.MasqResources.MasqUtils.velocityAutoController;
import static Library4997.MasqResources.MasqUtils.velocityTeleController;
import static Library4997.MasqResources.MasqUtils.xyAngleController;
import static Library4997.MasqResources.MasqUtils.xySpeedController;


/**
 * MasqRobot--> Contains all hardware and methods to runLinearOpMode the robot.
 *
 * Pullin' out the coupe at the lot
 * Told 'em "Fuck 12, fuck SWAT"
 * Bustin' all the bales out the box
 * I just hit a lick with the box
 * Had to put the stick in a box, mm
 * Pour up the whole damn seal, I'ma get lazy
 * I got the mojo-deals, we been trappin' like the '80's
 * She sucked a nigga soul, got the Cash App
 * Told 'em wipe a nigga nose, say slatt, slatt
 * I won't never sell my soul, and I can back that
 * And I really wanna know, where you at, at?
 * I was out back, where the stash at
 * Cruise the city in a bulletproof Cadillac (skrrt)
 * 'Cause I know these niggas after where the bag at
 * Gotta move smarter, gotta move harder
 * Niggas tryna get me for my water
 * I lay his ass down on my son or my daughter
 * I had the Draco with me, Dwayne Carter
 * 'Lotta niggas out here playin', ain't ballin'
 * I done put my whole arm in the rim, Vince Carter
 * And I know probably get a key for the quarter
 * Shawty barely seen in double C's I bought her
 * Got a bitch that's looking like Aaliyah, she a model
 * I got the pink slip, all my whips is key-less
 * Compton, I'm about to get the key to the city
 * Patek like the sea, forget it
 * Pullin' out the coupe at the lot
 * Told 'em "Fuck 12, fuck SWAT"
 * Bustin' all the bales out the box
 * I just hit a lick with the box
 * Had to put the stick in a box, mm
 * Pour up the whole damn seal, I'ma get lazy
 * I got the mojo-deals, we been trappin' like the '80's
 * She sucked a nigga soul, got the Cash App
 * Told 'em wipe a nigga nose, say slatt, slatt
 * I won't never sell my soul, and I can back that
 * And I really wanna know, where you at, at?
 * Ha-ha-ha, I been movin' 'em out
 * If Steelo with me, then he got the blues in the pouch
 * Took her to the forest, put the wood in her mouth
 * Bitch don't wear no shoes in my house
 * The private I'm flyin' in, I never wanna fly again
 * I take my chances in traffic
 * She suckin' on dick no hands with it
 * I just made the Rollie plain like a landing-strip
 * I'm a 2020 president candidate
 * I done put a hunnid bands on Zimmerman shit
 * I been movin' real gangsta, so that's why she pick a Crip
 * Shawty call me Chris Cole, 'cause I pop my shit
 * Got it out the mud, there's nothin' you can tell me, yeah
 * When I had the drugs, I was street-wealthy, yeah
 * Pullin' out the coupe at the lot
 * Told 'em "Fuck 12, fuck SWAT"
 * Bustin' all the bales out the box
 * I just hit a lick with the box
 * Had to put the stick in a box, mm
 * Pour up the whole damn seal, I'ma get lazy
 * I got the mojo-deals, we been trappin' like the '80's
 * She sucked a nigga soul, got the Cash App
 * Told 'em wipe a nigga nose, say slatt, slatt
 * I won't never sell my soul, and I can back that
 * And I really wanna know, where you at, at?
 *
 * I can swallow a bottle of alcohol and I'll feel like Godzilla
 * Better hit the deck like the card dealer
 * My whole squad's in here, walking around the party
 * A cross between a zombie apocalypse and big Bobby "The
 * Brain" Heenan which is probably the
 * Same reason I wrestle with mania
 * Shady's in this bitch, I'm posse'd up
 * Consider it to cross me a costly mistake
 * If they sleepin' on me, the hoes better get insomnia
 * ADHD, Hydroxycut
 * Pass the Courvoisi' (ayy, ayy)
 * In AA with an AK, melee, finna set it like a playdate
 * Better vacate, retreat like a vacay, mayday (ayy)
 * This beat is cray-cray, Ray J, H-A-H-A-H-A
 * Laughing all the way to the bank, I spray flames
 * They cannot tame or placate the
 * Monster (ayy)
 * You get in my way, I'ma feed you to the monster (yeah)
 * I'm normal during the day, but at night, turn to a monster (yeah)
 * When the moon shines like Ice Road Truckers
 * I look like a villain outta those blockbusters
 * Godzilla, fire spitter, monster
 * Blood on the dance floor, and on the Louis V carpet
 * Fire, Godzilla, fire, monster
 * Blood on the dance floor, and on the Louis V carpet
 * I'm just a product of Slick Rick and Onyx, told 'em, "Lick the balls"
 * Had 'em just appalled at so many things that pissed 'em off
 * It's impossible to list 'em all
 * And in the midst of all this
 * I'm in a mental hospital with a crystal ball
 * Tryna see, will I still be like this tomorrow?
 * Risperdal, voices whisper
 * My fist is balled back up against the wall, pencil drawn
 * This is just the song to go ballistic on
 * You just pulled a pistol on the guy with the missile launcher
 * I'm just a Loch Ness, the mythological
 * Quick to tell a bitch screw off like a fifth of Vodka
 * When you twist the top of the bottle, I'm a
 * Monster (ayy)
 * You get in my way, I'ma feed you to the monster (yeah)
 * I'm normal during the day, but at night, turn to a monster (yeah)
 * When the moon shines like Ice Road Truckers
 * I look like a villain outta those blockbusters
 * Godzilla, fire spitter, monster
 * Blood on the dance floor, and on the Louis V carpet
 * Fire, Godzilla, fire, monster
 * Blood on the dance floor, and on the Louis V carpet
 * If you never gave a damn, raise your hand
 * 'Cause I'm about to set trip, vacation plans
 * I'm on point like my index is, so all you will ever get is
 * The motherfuckin' finger (finger), prostate exam ('xam)
 * How can I have all these fans and perspire?
 * Like a liar's pants, I'm on fire
 * And I got no plans to retire and I'm still the man you admire
 * These chicks are spazzin' out, I only get more handsome and flier
 * I got 'em passin' out like what you do when you hand someone flyers
 * And what goes around comes around just like the blades on a chainsaw
 * 'Cause I caught the flap of my dollar stack right off the bat like a baseball
 * Like Kid Ink, bitch, I got them racks with so much ease that they call me Diddy
 * 'Cause I make bands and I call getting cheese a cakewalk (cheesecake) yeah
 * Bitch, I'm a player, I'm too motherfuckin' stingy for Cher
 * Won't even lend you an ear, ain't even pretending to care
 * But I tell a bitch I'll marry her if she'll bury her
 * Face on my genital area, the original Richard Ramirez
 * Christian Rivera, 'cause my lyrics never sit well
 * So they wanna give me the chair
 * Like a paraplegic, and it's scary, call it Harry Caray
 * 'Cause every Tom and Dick and Harry carry a Merriam motherfuckin' dictionary
 * Got 'em swearing up and down, they can't spit, this shit's hilarious
 * It's time to put these bitches in the obituary column
 * We wouldn't see eye to eye with a staring problem
 * Get the shaft like a steering column (Mark Jack)
 * Trigger happy, pack heat, but it's black ink
 * Evil half of the Bad Meets Evil
 * That means take a back seat
 * Take it back to Fat Petes with a maxi, single
 * Look at my rap sheet, what attracts these people
 * Is my gangster, bitch, like Apache with a catchy jingle
 * I stack chips, you barely got a half-eaten Cheeto
 * Fill 'em with the venom and eliminate 'em
 * Other words, I Minute Maid 'em
 * I don't wanna hurt 'em, but I did 'em in a fit of rage
 * I'm murderin' again, nobody will evade him
 * Finna kill 'em and dump all the fuckin' bodies in the lake
 * Obliterating everything, incinerate and renegade 'em
 * And I make anybody who want it with the pen afraid
 * But don't nobody want it, but they're gonna get it anyway
 * 'Cause I'm beginnin' to feel like I'm mentally ill
 * I'm Atilla, kill or be killed, I'm a killer, be the vanilla gorilla
 * You're bringin' the killer within me out of me
 * You don't want to be the enemy of the demon who went in me
 * Or being the recievin' enemy, what stupidity it'd be
 * Every bit of me is the epitome of a spitter
 * When I'm in the vicinity, motherfucker, you better duck
 * Or you finna be dead the minute you run into me
 * A hunnid percent of you is a fifth of a percent of me
 * I'm 'bout to fuckin' finish you bitch, I'm unfadable
 * You wanna battle, I'm available, I'm blowin' up like an inflatable
 * I'm undebatable, I'm unavoidable, I'm unevadable
 * I'm on the toilet bowl, I got a trailer full of money and I'm paid in full
 * I'm not afraid to pull the
 * Man, stop
 * Look what I'm plannin', haha
 */

/*
TODO:
    Path Control
    Unit Tests for all major functions
    State Machine support
 */
public abstract class MasqRobot {
    public abstract void mapHardware(HardwareMap hardwareMap);
    public abstract void init(HardwareMap hardwareMap) throws InterruptedException;

    public MasqMechanumDriveTrain driveTrain;
    public MasqPositionTracker tracker;
    public DashBoard dash;
    private MasqClock timeoutClock = new MasqClock();

    public static boolean opModeIsActive() {return MasqUtils.opModeIsActive();}

    public void strafe(double distance, double angle, double timeout, double speed) {
        driveTrain.resetEncoders();
        double targetClicks = distance * driveTrain.getEncoder().getClicksPerInch();
        double clicksRemaining;
        double power, angularError, targetAngle = tracker.getHeading(), powerAdjustment;
        timeoutClock.reset();
        do {
            clicksRemaining = targetClicks - Math.abs(driveTrain.getCurrentPositionPositive());
            power = driveController.getOutput(clicksRemaining) * speed;
            power = Range.clip(power, -1.0, +1.0);
            angularError = MasqUtils.adjustAngle(targetAngle - tracker.getHeading());
            powerAdjustment = angleController.getOutput(MasqUtils.adjustAngle(angularError));
            powerAdjustment = Range.clip(powerAdjustment, -1.0, +1.0);
            driveTrain.setVelocityMECH(angle, power, tracker.getHeading(), powerAdjustment);
            dash.create("ERROR: ", clicksRemaining);
            dash.create("HEADING: ", tracker.getHeading());
            dash.update();
        } while (opModeIsActive() && !timeoutClock.elapsedTime(timeout, MasqClock.Resolution.SECONDS) && (Math.abs(angularError) > 5 || clicksRemaining/targetClicks > 0.01));
        driveTrain.setVelocity(0);
        MasqUtils.sleep(MasqUtils.DEFAULT_SLEEP_TIME);
    }
    public void strafe(double distance, double angle, double timeout) {
        strafe(distance, angle, timeout, 0.7);
    }
    public void strafe (double distance, double angle) {
        strafe(distance, angle, DEFAULT_TIMEOUT);
    }

    public void drive(double distance, double speed, Direction direction, double timeout, double sleepTime) {
        driveTrain.resetEncoders();
        double targetAngle = tracker.getHeading();
        double targetClicks = distance * driveTrain.getEncoder().getClicksPerInch();
        double clicksRemaining;
        double angularError, powerAdjustment, power, leftPower, rightPower, maxPower;
        do {
            clicksRemaining = targetClicks - Math.abs(driveTrain.getCurrentPosition());
            power = driveController.getOutput(clicksRemaining) * speed;
            angularError = MasqUtils.adjustAngle(targetAngle - tracker.getHeading());
            powerAdjustment = angleController.getOutput(MasqUtils.adjustAngle(angularError));
            leftPower = (direction.value * power) - powerAdjustment;
            rightPower = (direction.value * power) + powerAdjustment;
            maxPower = MasqUtils.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }
            driveTrain.setVelocity(leftPower, rightPower);
            dash.create("LEFT POWER: ", leftPower);
            dash.create("RIGHT POWER: ", rightPower);
            dash.create("ERROR: ", clicksRemaining);
            dash.create("HEADING: ", tracker.getHeading());
            dash.update();
        } while (opModeIsActive() && !timeoutClock.elapsedTime(timeout, MasqClock.Resolution.SECONDS) && (Math.abs(angularError) > 5 || clicksRemaining/targetClicks > 0.01));
        driveTrain.setVelocity(0);
        MasqUtils.sleep(sleepTime);
    }
    public void drive(double distance, double speed, Direction strafe, double timeout) {
        drive(distance, speed, strafe, timeout, MasqUtils.DEFAULT_SLEEP_TIME);
    }
    public void drive(double distance, double speed, Direction strafe) {
        drive(distance, speed, strafe, MasqUtils.DEFAULT_TIMEOUT);
    }
    public void drive(double distance, Direction direction, double timeout) {
        drive(distance, 1, direction, timeout);
    }
    public void drive(double distance, double speed){drive(distance, speed, Direction.FORWARD);}
    public void drive(double distance, Direction direction) {drive(distance, 0.5, direction);}
    public void drive(double distance) {drive(distance, 0.5);}

    public void driveAbsoluteAngle(double distance, double angle, double speed, double timeout, double sleepTime) {
        driveTrain.resetEncoders();
        double targetClicks = distance * driveTrain.getEncoder().getClicksPerInch();
        double clicksRemaining;
        double angularError, powerAdjustment, power, leftPower, rightPower, maxPower;
        timeoutClock.reset();
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPosition()));
            power = driveController.getOutput(clicksRemaining) * speed;
            angularError = MasqUtils.adjustAngle(angle - tracker.getHeading());
            powerAdjustment = angleController.getOutput(angularError);
            leftPower = power - powerAdjustment;
            rightPower = power + powerAdjustment;
            maxPower = MasqUtils.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }
            driveTrain.setVelocity(leftPower, rightPower);
            dash.create("LEFT POWER: ", leftPower);
            dash.create("RIGHT POWER: ", rightPower);
            dash.create("ERROR: ", clicksRemaining);
            dash.update();
        } while (opModeIsActive() && !timeoutClock.elapsedTime(timeout, MasqClock.Resolution.SECONDS) && ((clicksRemaining / targetClicks) > 0.01));
        driveTrain.setVelocity(0);
        MasqUtils.sleep(sleepTime);
    }
    public void driveAbsoluteAngle(double distance, double angle, double speed, double timeout) {
        driveAbsoluteAngle(distance, angle, speed, timeout, MasqUtils.DEFAULT_SLEEP_TIME);
    }
    public void driveAbsoluteAngle(double distance, double angle, double speed) {
        driveAbsoluteAngle(distance, angle, speed, MasqUtils.DEFAULT_TIMEOUT);
    }
    public void driveAbsoluteAngle(double distance, double angle){
        driveAbsoluteAngle(distance, angle, 0.5);
    }

    public void turnRelative(double angle, double timeout, double sleepTime) {
        double targetAngle = MasqUtils.adjustAngle(angle + tracker.getHeading());
        double acceptableError = .5;
        double error = MasqUtils.adjustAngle(targetAngle - tracker.getHeading());
        double power;
        timeoutClock.reset();
        while (opModeIsActive() && (MasqUtils.adjustAngle(Math.abs(error)) > acceptableError)
                && !timeoutClock.elapsedTime(timeout, MasqClock.Resolution.SECONDS)) {
            error = MasqUtils.adjustAngle(targetAngle - tracker.getHeading());
            power = turnController.getOutput(error);
            if (Math.abs(power) >= 1) power /= Math.abs(power);
            driveTrain.setVelocity(power, -power);
            dash.create("TargetAngle", targetAngle);
            dash.create("Heading", tracker.getHeading());
            dash.create("AngleLeftToCover", error);
            dash.create("Power: ", power);
            dash.create("Raw Power: ", driveTrain.getPower());
            dash.update();
        }
        driveTrain.setVelocity(0);
        MasqUtils.sleep(sleepTime);
    }
    public void turnRelative(double angle, double timeout) {
        turnRelative(angle, timeout, DEFAULT_SLEEP_TIME);
    }
    public void turnRelative(double angle) {
        turnRelative(angle, DEFAULT_TIMEOUT);
    }

    public void turnAbsolute(double angle,  double timeout, double sleepTime) {
        double acceptableError = 2;
        double error = MasqUtils.adjustAngle(angle - tracker.getHeading());
        double power;
        timeoutClock.reset();
        while (opModeIsActive() && (MasqUtils.adjustAngle(Math.abs(error)) > acceptableError)
                && !timeoutClock.elapsedTime(timeout, MasqClock.Resolution.SECONDS)) {
            error = MasqUtils.adjustAngle(angle - tracker.getHeading());
            power = turnController.getOutput(error);
            if (Math.abs(power) >= 1) power /= Math.abs(power);
            driveTrain.setVelocity(-power, power);
            dash.create("RIGHT POWER: " ,power);
            dash.create("TargetAngle", angle);
            dash.create("Heading", tracker.getHeading());
            dash.create("AngleLeftToCover", error);
            dash.update();
        }
        driveTrain.setVelocity(0);
        MasqUtils.sleep(sleepTime);
    }
    public void turnAbsolute(double angle, double timeout) {
        turnAbsolute(angle, timeout,DEFAULT_SLEEP_TIME);
    }
    public void turnAbsolute(double angle) {
        turnAbsolute(angle, DEFAULT_TIMEOUT);
    }

    public void stop(MasqPredicate stopCondition, double angle, double speed, Direction direction, double timeout) {
        driveTrain.resetEncoders();
        double angularError, powerAdjustment, power, leftPower, rightPower, maxPower;
        timeoutClock.reset();
        do {
            power = direction.value * speed;
            angularError = MasqUtils.adjustAngle(angle - tracker.getHeading());
            powerAdjustment = angleController.getOutput(angularError);
            powerAdjustment *= direction.value;
            leftPower = power - powerAdjustment;
            rightPower = power + powerAdjustment;
            maxPower = MasqUtils.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }
            driveTrain.setVelocity(leftPower, rightPower);
            dash.create("LEFT POWER: ",leftPower);
            dash.create("RIGHT POWER: ",rightPower);
            dash.create("Angle Error", angularError);
            dash.update();
        } while (opModeIsActive() && !timeoutClock.elapsedTime(timeout, MasqClock.Resolution.SECONDS) && stopCondition.run());

        driveTrain.setVelocity(0);
    }
    public void stop(MasqPredicate stopCondition, double angle, double speed, Direction direction) {
        stop(stopCondition, angle, speed, direction, MasqUtils.DEFAULT_TIMEOUT);
    }
    public void stop(MasqPredicate sensor, double angle, double power) {
        stop(sensor, angle, power, Direction.FORWARD);
    }
    public void stop(MasqPredicate stopCondition, double angle) {
        stop(stopCondition, angle, 0.5);
    }
    public void stop(MasqPredicate sensor){
        stop(sensor, tracker.getHeading());
    }
    public void stop(MasqPredicate stopCondition, int timeout) {
        stop(stopCondition, tracker.getHeading(), 0.5, Direction.FORWARD, timeout);
    }

    public void xyPath(double timeout, MasqWayPoint... points) {
        MasqMechanumDriveTrain.angleCorrectionController.setKp(xyAngleController.getKp());
        MasqMechanumDriveTrain.angleCorrectionController.setKi(xyAngleController.getKi());
        MasqMechanumDriveTrain.angleCorrectionController.setKd(xyAngleController.getKd());
        List<MasqWayPoint> pointsWithRobot = new ArrayList<>(Arrays.asList(points));
        pointsWithRobot.add(0, getCurrentWayPoint());

        MasqPIDController travelAngleController = new MasqPIDController(0.01, 0, 0);

        int index = 1;

        MasqClock pointTimeout = new MasqClock();
        timeoutClock.reset();
        while (!timeoutClock.elapsedTime(timeout, MasqClock.Resolution.SECONDS) &&
                index < pointsWithRobot.size()) {
            double lookAheadDistance = pointsWithRobot.get(index).getLookAhead();
            travelAngleController.setKp(pointsWithRobot.get(index).getAngularCorrectionSpeed());
            MasqMechanumDriveTrain.angleCorrectionController.setKp(pointsWithRobot.get(index).getAngularCorrectionSpeed());
            xySpeedController.setKp(pointsWithRobot.get(index).getDriveCorrectionSpeed());
            MasqVector target = new MasqVector(pointsWithRobot.get(index).getX(), pointsWithRobot.get(index).getY());
            MasqVector current = new MasqVector(tracker.getGlobalX(), tracker.getGlobalY());
            MasqVector initial = new MasqVector(pointsWithRobot.get(index - 1).getX(), pointsWithRobot.get(index - 1).getY());
            double speed = 1;
            pointTimeout.reset();
            while (!pointTimeout.elapsedTime(pointsWithRobot.get(index).getTimeout(), MasqClock.Resolution.SECONDS) &&
                    !current.equal(pointsWithRobot.get(index).getTargetRadius(), target) && opModeIsActive() && speed > 0.1) {
                double heading = Math.toRadians(tracker.getHeading());
                MasqVector headingUnitVector = new MasqVector(Math.sin(heading), Math.cos(heading));
                MasqVector lookahead = MasqUtils.getLookAhead(initial, current, target, lookAheadDistance);
                MasqVector pathDisplacement = initial.displacement(target);
                boolean closerThanLookAhead = initial.displacement(lookahead).getMagnitude() > pathDisplacement.getMagnitude();
                boolean approachingFinalPos = index == pointsWithRobot.size() - 1;
                if (closerThanLookAhead) {
                    if (approachingFinalPos) lookahead = new MasqVector(target.getX(), target.getY());
                    else break;
                }
                MasqVector lookaheadDisplacement = current.displacement(lookahead);
                double pathAngle = MasqUtils.adjustAngle(headingUnitVector.angleTan(lookaheadDisplacement));
                speed = xySpeedController.getOutput(current.displacement(target).getMagnitude());
                double maxVelocity = pointsWithRobot.get(index).getMaxVelocity();
                speed = scaleNumber(speed, pointsWithRobot.get(index).getMinVelocity(), maxVelocity);
                double powerAdjustment = travelAngleController.getOutput(pathAngle);
                double leftPower = speed + powerAdjustment;
                double rightPower = speed - powerAdjustment;

                MasqWayPoint.PointMode mode = pointsWithRobot.get(index).getSwitchMode();
                boolean mechMode = approachingFinalPos ||
                        (current.equal(pointsWithRobot.get(index).getModeSwitchRadius(), target) && mode == SWITCH) ||
                        mode == MECH;

                if (mechMode) {
                    pathAngle = 90 - Math.toDegrees(Math.atan2(lookaheadDisplacement.getY(), lookaheadDisplacement.getX()));
                    driveTrain.setVelocityMECH(
                            pathAngle + tracker.getHeading(), speed,
                            pointsWithRobot.get(index).getH()
                    );
                }
                else driveTrain.setVelocity(leftPower, rightPower);

                tracker.updateSystem();
                dash.create(current.setName("Current Point"));
                dash.create("Heading: ", Math.toDegrees(heading));
                dash.create("Angular Correction Speed: ", MasqMechanumDriveTrain.angleCorrectionController.getKp());
                dash.create("Drive Correction Speed: ", pointsWithRobot.get(index).getDriveCorrectionSpeed());
                dash.update();
                current = new MasqVector(tracker.getGlobalX(), tracker.getGlobalY());
            }
            index++;
        }
        driveTrain.setVelocity(0);
    }
    public void xyPathV2(double timeout, MasqWayPoint... points) {
        MasqMechanumDriveTrain.angleCorrectionController.setKp(xyAngleController.getKp());
        MasqMechanumDriveTrain.angleCorrectionController.setKi(xyAngleController.getKi());
        MasqMechanumDriveTrain.angleCorrectionController.setKd(xyAngleController.getKd());
        List<MasqWayPoint> pointsWithRobot = new ArrayList<>(Arrays.asList(points));
        pointsWithRobot.add(0, getCurrentWayPoint());
        MasqPIDController travelAngleController = new MasqPIDController(0.01, 0, 0);
        int index = 1;
        MasqClock pointTimeout = new MasqClock();
        timeoutClock.reset();
        while (!timeoutClock.elapsedTime(timeout, MasqClock.Resolution.SECONDS) &&
                index < pointsWithRobot.size()) {
            double lookAheadDistance = pointsWithRobot.get(index).getLookAhead();
            travelAngleController.setKp(pointsWithRobot.get(index).getAngularCorrectionSpeed());
            MasqMechanumDriveTrain.angleCorrectionController.setKp(pointsWithRobot.get(index).getAngularCorrectionSpeed());
            MasqVector target = new MasqVector(pointsWithRobot.get(index).getX(), pointsWithRobot.get(index).getY());
            MasqVector current = new MasqVector(tracker.getGlobalX(), tracker.getGlobalY());
            MasqVector initial = new MasqVector(pointsWithRobot.get(index - 1).getX(), pointsWithRobot.get(index - 1).getY());
            double speed = 1;
            pointTimeout.reset();
            while (!pointTimeout.elapsedTime(pointsWithRobot.get(index).getTimeout(), MasqClock.Resolution.SECONDS) &&
                    !current.equal(pointsWithRobot.get(index).getTargetRadius(), target) && opModeIsActive() && speed > 0.1) {
                double heading = Math.toRadians(tracker.getHeading());
                // Y and X components are reversed because the axis are switched for the robot and a cartesian coordinate plane, where 0 degrees is east.
                // This robot should have 0 degrees at north.
                MasqVector headingUnitVector = new MasqVector(Math.sin(heading), Math.cos(heading));
                MasqVector lookahead = MasqUtils.getLookAhead(initial, current, target, lookAheadDistance);
                MasqVector pathDisplacement = initial.displacement(target);
                boolean closerThanLookAhead = initial.displacement(lookahead).getMagnitude() > pathDisplacement.getMagnitude();
                boolean approachingFinalPos = index == pointsWithRobot.size() - 1;
                if (closerThanLookAhead) {
                    if (approachingFinalPos) lookahead = new MasqVector(target.getX(), target.getY());
                    else break;
                }
                MasqVector lookaheadDisplacement = current.displacement(lookahead);
                double pathAngle = MasqUtils.adjustAngle(headingUnitVector.angleTan(lookaheadDisplacement));
                speed = xySpeedController.getOutput(current.displacement(target).getMagnitude());
                double maxVelocity = pointsWithRobot.get(index).getMaxVelocity();
                speed = scaleNumber(speed, pointsWithRobot.get(index).getMinVelocity(), maxVelocity);
                double powerAdjustment = travelAngleController.getOutput(pathAngle);


                MasqWayPoint.PointMode mode = pointsWithRobot.get(index).getSwitchMode();
                boolean mechMode = approachingFinalPos ||
                        (current.equal(pointsWithRobot.get(index).getModeSwitchRadius(), target) && mode == SWITCH) ||
                        mode == MECH;

                if (mechMode) {
                    pathAngle = 90 - Math.toDegrees(Math.atan2(lookaheadDisplacement.getY(), lookaheadDisplacement.getX()));
                    driveTrain.setVelocityMECH(
                            pathAngle + tracker.getHeading(), speed,
                            -pointsWithRobot.get(index).getH()
                    );
                }
                else {
                    double pathAngleScaled = 1 - (Math.abs(pathAngle) / 180);
                    speed = speed * pathAngleScaled * pointsWithRobot.get(index).getSpeedBias();
                    double leftPower = speed + powerAdjustment;
                    double rightPower = speed - powerAdjustment;
                    driveTrain.setVelocity(leftPower, rightPower);
                }

                dash.create("pathAngle: ", pathAngle);

                tracker.updateSystem();
                current = new MasqVector(tracker.getGlobalX(), tracker.getGlobalY());
                int i = 0;
                for (MasqWayPoint point : pointsWithRobot) {
                    dash.create(point.setName("Index: " + i));
                    i++;
                }
                dash.update();
            }
            if (pointsWithRobot.get(index).getOnComplete() != null)
                pointsWithRobot.get(index).getOnComplete().run();
            index++;
        }
        driveTrain.setVelocity(0);
    }

    public void NFS(MasqController c) {
        float move = -c.leftStickY();
        float turn = c.rightStickX() * 0.7f;
        double left = move + turn;
        double right = move - turn;
        double max = MasqUtils.max(left, right);
        if(max > 1.0) {
            left /= max;
            right /= max;
        }
        driveTrain.setPower(left, right);
    }

    public void TANK(MasqController c) {
        double left = -c.leftStickY();
        double right = -c.rightStickY();
        double leftRate = driveTrain.leftDrive.getVelocity();
        double rightRate = driveTrain.rightDrive.getVelocity();
        double maxRate = MasqUtils.max(Math.abs(leftRate/left), Math.abs(rightRate/right));
        leftRate /= maxRate;
        rightRate /= maxRate;
        double leftError =  left - leftRate;
        double rightError = right - rightRate;
        driveTrain.rightDrive.setPower(right);
        driveTrain.leftDrive.setPower(left);
    }

    public void MECH(MasqController c, Direction direction, boolean fieldCentric, double speedMultiplier, double turnMultiplier) {
        int disable = 0;
        if (fieldCentric) disable = 1;

        double angle;

        double x = -c.leftStickY();
        double y = c.leftStickX();
        double xR = c.rightStickX();

        angle = Math.atan2(y, x) + (Math.toRadians(tracker.getHeading()) * disable);
        double adjustedAngle = angle + Math.PI/4;

        double speedMagnitude = Math.hypot(x, y);

        double leftFront = (Math.sin(adjustedAngle) * speedMagnitude * speedMultiplier) + xR * turnMultiplier * direction.value;
        double leftBack = (Math.cos(adjustedAngle) * speedMagnitude * speedMultiplier) + xR  * turnMultiplier * direction.value;
        double rightFront = (Math.cos(adjustedAngle) * speedMagnitude * speedMultiplier) - xR * turnMultiplier * direction.value;
        double rightBack = (Math.sin(adjustedAngle) * speedMagnitude * speedMultiplier) - xR * turnMultiplier * direction.value;

        double max = MasqUtils.max(Math.abs(leftFront), Math.abs(leftBack), Math.abs(rightFront), Math.abs(rightBack));
        if (max > 1) {
            leftFront /= Math.abs(max);
            leftBack /= Math.abs(max);
            rightFront /= Math.abs(max);
            rightBack /= Math.abs(max);
        }

        driveTrain.leftDrive.motor1.setVelocity(leftFront * direction.value);
        driveTrain.leftDrive.motor2.setVelocity(leftBack * direction.value);
        driveTrain.rightDrive.motor1.setVelocity(rightFront * direction.value);
        driveTrain.rightDrive.motor2.setVelocity(rightBack * direction.value);
    }
    public void MECH(MasqController c, Direction direction) {
        MECH(c, direction, false, MasqUtils.DEFAULT_SPEED_MULTIPLIER, MasqUtils.DEFAULT_TURN_MULTIPLIER);
    }
    public void MECH(MasqController c, boolean fieldCentric) {
        MECH(c, Direction.FORWARD, fieldCentric, MasqUtils.DEFAULT_SPEED_MULTIPLIER, MasqUtils.DEFAULT_TURN_MULTIPLIER);
    }
    public void MECH(MasqController c) {
        MECH(c, Direction.FORWARD, false, MasqUtils.DEFAULT_SPEED_MULTIPLIER, MasqUtils.DEFAULT_TURN_MULTIPLIER);
    }
    public void MECH(MasqController c, double speedMutliplier, double turnMultiplier) {
        MECH(c, Direction.FORWARD, false, speedMutliplier, turnMultiplier);
    }

    public void initializeTeleop(){
        driveTrain.setKp(velocityTeleController.getKp());
        driveTrain.setKi(velocityTeleController.getKi());
        driveTrain.setKd(velocityTeleController.getKd());
    }
    public void initializeAutonomous() {
        driveTrain.setKp(velocityAutoController.getKp());
        driveTrain.setKi(velocityAutoController.getKi());
        driveTrain.setKd(velocityAutoController.getKd());
    }
    private MasqWayPoint getCurrentWayPoint() {
        return new MasqWayPoint().setPoint(new MasqPoint(tracker.getGlobalX(), tracker.getGlobalY(), tracker.getHeading())).setName("Inital WayPoint");
    }
}