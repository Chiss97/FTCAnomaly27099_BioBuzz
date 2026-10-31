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

    // Acceleration limiting (only when speeding up, slowing down is always instant)

    // How fast drive speed can increase, in full power per second.
    // Time to reach full speed = 1.0 / ACCEL_RATE, so 1.0 / 2.0 = 0.5 seconds.
    private static final double ACCEL_RATE = 2.0;

    // How fast turning speed can increase, in full power per second.
    // Time to reach full turning = 1.0 / ROTATION_RATE, so 1.0 / 2.0 = 0.5 seconds.
    private static final double ROTATION_RATE = 2.0;

    private double curForward = 0, curStrafe = 0, curRot = 0;
    private long lastNanos = System.nanoTime();

    public double[] out = new double[3]; // Forward, strafe, turn

    public double shape(double x) {
        if (Math.abs(x) < DEADZONE) return 0;
        double s = (Math.abs(x) - DEADZONE) / (1 - DEADZONE); // Rescale past deadzone
        double shaped = LINEAR_WEIGHT * s + (1 - LINEAR_WEIGHT) * Math.pow(s, EXPONENT);
        return Math.signum(x) * shaped;
    }

    private static double step(double current, double target, double maxStep) {
        return current + Range.clip(target - current, -maxStep, maxStep);
    }

    public void update(double forward, double strafe, double turn) {
        long now = System.nanoTime();
        double dt = Math.min((now - lastNanos) / 1e9, 0.1); // Cap so a lag spike can't jump
        lastNanos = now;

        // Driving: always goes exactly where the stick points, only the speed is limited
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

        // Turning: never turns the opposite way from the stick, only the speed is limited
        if (turn * curRot < 0) {
            curRot = 0; // Switching turn direction: stop turning instantly first
        }
        if (Math.abs(turn) < Math.abs(curRot)) {
            curRot = turn; // Slowing down is instantaneous
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