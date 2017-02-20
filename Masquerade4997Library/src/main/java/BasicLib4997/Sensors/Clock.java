package BasicLib4997.Sensors;

import java.util.Locale;


public class Clock {

    private long startTime;

    private boolean isPaused = false;
    private long pauseStart;
    private long pauseLength;

    private String name = "Timer";


    public Clock() {this.reset();}
    public Clock(String n) {setName(n); this.reset();}


    public void reset() {
        isPaused = false;
        pauseStart = 0L;
        pauseLength = 0L;
        startTime = System.nanoTime();
    }

    public long nanoseconds() {return System.nanoTime() - startTime - pauseLength;}
    public double milliseconds() {return nanoseconds() * 1E-6;}
    public double seconds() {return nanoseconds() * 1E-9;}


    public enum Resolution {

        NANOSECONDS (1),
        MILLISECONDS (1E6),
        SECONDS (1E9);

        private final double multiplier;

        Resolution (double multiplier) {this.multiplier = multiplier;}

    }


    public boolean elapsedTime(double time, Resolution resolution) {
        return nanoseconds() > (long)(time * resolution.multiplier);
    }

    public void pause() {
        if (!isPaused) pauseStart = System.nanoTime();
        isPaused = true;
    }

    public void resume() {
        if (isPaused) pauseLength += (System.nanoTime() - pauseStart);
        isPaused = false;
    }

    public boolean isPaused() {return isPaused;}


    public Clock setName (String n) {name = n; return this;}
    public String getName() {return name;}

    public String telemetryRun() {
        return String.format(Locale.US, "%.5f  %s", seconds(), isPaused() ? "PAUSED" : "");
    }

}