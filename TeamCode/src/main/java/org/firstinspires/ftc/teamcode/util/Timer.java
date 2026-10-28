package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Timer {
    private static final ElapsedTime matchTimer = new ElapsedTime();

    // Call this once right after waitForStart() in any OpMode
    public static void start() {
        matchTimer.reset();
    }

    // Returns seconds elapsed since match start
    public static double seconds() {
        return matchTimer.seconds();
    }

    public static double milliseconds() {
        return matchTimer.milliseconds();
    }
}
