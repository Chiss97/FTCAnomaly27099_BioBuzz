package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.Range;

public class DriveInputFilter {

    // Input shaping

    // How far the stick has to move before the robot reacts (0.05 = 5% of the way).
    // Raise it if the robot creeps when you let go of the sticks.
    private static final double DEADZONE = 0.05;

    // Where the precision zone ends and the fillet starts (0.20 = 20% of the stick).
    private static final double LINEAR_END = 0.20;

    // Where the fillet ends and the power zone starts (0.30 = 30% of the stick).
    // A wider gap from LINEAR_END gives a gentler bend.
    private static final double POWER_START = 0.30;

    // The power zone curve is power = POWER_OFFSET + (1 - POWER_OFFSET) * stick^EXPONENT.
    // It always reaches exactly full power at full stick, so no stick travel is wasted.
    // Higher EXPONENT = flatter middle and a steeper climb at the end.
    private static final double EXPONENT = 4;

    // Everything below is calculated automatically from the settings above. Don't edit these.

    private static final double FILLET_WIDTH = POWER_START - LINEAR_END;

    // The power offset is picked so the fillet is one clean bend with no wiggle.
    // That happens when the fillet's average slope is halfway between the slope
    // it starts with (1) and the slope it ends with (the power zone's slope).
    private static final double POWER_OFFSET;
    static {
        double s0 = Math.pow(POWER_START, EXPONENT);                     // stick^EXPONENT at the joint
        double s1 = EXPONENT * Math.pow(POWER_START, EXPONENT - 1);      // its slope at the joint
        POWER_OFFSET = (FILLET_WIDTH / 2 * (1 + s1) + LINEAR_END - s0)
                / (1 - s0 + FILLET_WIDTH * s1 / 2);
    }

    // The fillet ends match both zones in height and slope, so there are no corners.
    private static final double FILLET_START_HEIGHT = LINEAR_END; // precision zone is y = x
    private static final double FILLET_START_SLOPE = 1.0;
    private static final double FILLET_END_HEIGHT = powerZone(POWER_START);
    private static final double FILLET_END_SLOPE = (1 - POWER_OFFSET) * EXPONENT * Math.pow(POWER_START, EXPONENT - 1);

    // Acceleration limiting (only when speeding up, slowing down is always instant)

    // How fast drive speed can increase, in full power per second.
    // Time to reach full speed = 1.0 / ACCEL_RATE, so 1.0 / 4.0 = 0.25 seconds.
    private static final double ACCEL_RATE = 4.0;

    // How fast turning speed can increase, in full power per second.
    // Time to reach full turning = 1.0 / ROTATION_RATE, so 1.0 / 4.0 = 0.25 seconds.
    private static final double ROTATION_RATE = 4.0;

    private double curForward = 0, curStrafe = 0, curRot = 0;
    private long lastNanos = System.nanoTime();

    public double[] out = new double[3]; // Forward, strafe, turn

    // Custom response curve with a precision zone, a fillet, and a power zone
    public double shape(double x) {
        double a = Math.abs(x);
        if (a < DEADZONE) return 0;

        double y;
        if (a < LINEAR_END) {
            y = a; // Precision zone, power equals stick position
        } else if (a <= POWER_START) {
            y = fillet(a); // Smooth joint between the two zones
        } else {
            y = powerZone(a); // Power zone
        }
        return Math.signum(x) * y; // Same curve for pushing and pulling
    }

    // Power zone, starts near POWER_OFFSET and reaches exactly 1.0 at full stick
    private static double powerZone(double a) {
        return POWER_OFFSET + (1 - POWER_OFFSET) * Math.pow(a, EXPONENT);
    }

    // Cubic that starts at the end of the precision zone and ends at the start of the
    // power zone, matching the height and slope of both (a cubic Hermite curve).
    private static double fillet(double a) {
        double t = (a - LINEAR_END) / FILLET_WIDTH; // 0 at the start of the fillet, 1 at the end
        double t2 = t * t;
        double t3 = t2 * t;
        return (2 * t3 - 3 * t2 + 1) * FILLET_START_HEIGHT
                + (t3 - 2 * t2 + t) * FILLET_WIDTH * FILLET_START_SLOPE
                + (-2 * t3 + 3 * t2) * FILLET_END_HEIGHT
                + (t3 - t2) * FILLET_WIDTH * FILLET_END_SLOPE;
    }

    private static double step(double current, double target, double maxStep) {
        return current + Range.clip(target - current, -maxStep, maxStep);
    }

    public void update(double forward, double strafe, double turn) {
        long now = System.nanoTime();
        double dt = Math.min((now - lastNanos) / 1e9, 0.1); // Cap so a lag spike can't jump
        lastNanos = now;

        // Driving always goes exactly where the stick points, only the speed is limited
        double stickLength = Math.hypot(forward, strafe);
        if (stickLength == 0) {
            curForward = 0; // Stick let go makes it stop instantly
            curStrafe = 0;
        } else {
            // The stick's direction, as a length-1 arrow
            double dirForward = forward / stickLength;
            double dirStrafe = strafe / stickLength;

            double targetSpeed = Math.min(stickLength, 1.0);

            // How much of the robot's current motion already goes in the stick's direction
            // (0 if it was going sideways to it or the opposite way)
            double speedAlongStick = Math.max(0, curForward * dirForward + curStrafe * dirStrafe);

            // Speeding up is limited, slowing down is instant
            double speed = Math.min(targetSpeed, speedAlongStick + ACCEL_RATE * dt);

            curForward = dirForward * speed;
            curStrafe = dirStrafe * speed;
        }

        // Turning never goes the opposite way from the stick, only the speed is limited
        if (turn * curRot < 0) {
            curRot = 0; // When switching turn direction, stop turning instantly first
        }
        if (Math.abs(turn) < Math.abs(curRot)) {
            curRot = turn; // Slowing down is instant
        } else {
            curRot = step(curRot, turn, ROTATION_RATE * dt); // Speeding up is limited
        }

        out[0] = curForward;
        out[1] = curStrafe;
        out[2] = curRot;
    }

    public void reset() {
        curForward = 0;
        curStrafe = 0;
        curRot = 0;
        lastNanos = System.nanoTime();
    }
}