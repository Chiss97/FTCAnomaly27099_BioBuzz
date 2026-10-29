package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.Range;

public class DriveInputFilter {

    // Input shaping

    // How far the stick has to move before the robot reacts (0.05 = 5% of the way).
    // Raise it if the robot creeps when you let go of the sticks.
    private static final double DEADZONE = 0.05;

    // How strong the curve is. 1 = no curve, higher = more control near the center
    // but less response in the middle of the stick.
    private static final double EXPONENT = 3;

    // How much of the output is a straight line vs the curve (0.3 = 30% straight, 70% curve).
    // Raise it if small stick movements feel too weak.
    private static final double LINEAR_WEIGHT = 0.3;

    // Acceleration smoothing

    // How fast drive speed can change, in full power per second.
    // Time to reach full speed (or stop from it) = 1.0 / MAGNITUDE_RATE, so 1.0 / 4.0 = 0.25 seconds.
    private static final double MAGNITUDE_RATE = 4.0;

    // How fast the driving direction can swing while moving, in degrees per second.
    // Time for a turn of X degrees = X / DIRECTION_RATE_DEG, so a 90 degree change takes 90 / 450 = 0.2 seconds.
    private static final double DIRECTION_RATE_DEG = 450;

    // How fast turning speed can change, in full power per second.
    // Time to reach full turning = 1.0 / ROTATION_RATE, so 1.0 / 6.0 = about 0.17 seconds.
    private static final double ROTATION_RATE = 6.0;

    private double curMag = 0, curDir = 0, curRot = 0;
    private long lastNanos = System.nanoTime();

    public double[] out = new double[3]; // forward, strafe, turn

    public double shape(double x) {
        if (Math.abs(x) < DEADZONE) return 0;
        double s = (Math.abs(x) - DEADZONE) / (1 - DEADZONE); // rescale past deadzone
        double shaped = LINEAR_WEIGHT * s + (1 - LINEAR_WEIGHT) * Math.pow(s, EXPONENT);
        return Math.signum(x) * shaped;
    }

    private static double wrap(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }

    private static double step(double current, double target, double maxStep) {
        return current + Range.clip(target - current, -maxStep, maxStep);
    }

    public void update(double forward, double strafe, double turn) {
        long now = System.nanoTime();
        double dt = Math.min((now - lastNanos) / 1e9, 0.1); // cap so a lag spike can't jump
        lastNanos = now;

        double rawMag = Math.min(Math.hypot(forward, strafe), 1.0);
        double rawDir = Math.atan2(strafe, forward);

        double targetMag = rawMag;

        if (rawMag > 0) {
            double diff = wrap(rawDir - curDir);
            if (curMag < 0.05) {
                curDir = rawDir; // nearly stopped, go straight where the stick points
            } else if (Math.abs(diff) > Math.toRadians(153)) {
                targetMag = 0; // big reversal (over 153 degrees), slow down first, then snap
            } else {
                double maxDirStep = Math.toRadians(DIRECTION_RATE_DEG) * dt;
                curDir = wrap(curDir + Range.clip(diff, -maxDirStep, maxDirStep));
            }
        }
        curMag = step(curMag, targetMag, MAGNITUDE_RATE * dt);
        curRot = step(curRot, turn, ROTATION_RATE * dt);

        out[0] = curMag * Math.cos(curDir);
        out[1] = curMag * Math.sin(curDir);
        out[2] = curRot;
    }

    public void reset() {
        curMag = 0;
        curDir = 0;
        curRot = 0;
        lastNanos = System.nanoTime();
    }
}