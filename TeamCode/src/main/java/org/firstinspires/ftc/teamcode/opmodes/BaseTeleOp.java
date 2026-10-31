package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.Hardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.Timer;

public abstract class BaseTeleOp extends LinearOpMode {
    protected Hardware robotHardware;
    protected Intake intake;
    protected GoBildaPinpointDriver pinpoint;
    protected GamepadEx controller1;
    protected TriggerReader lt1Reader;
    protected TriggerReader rt1Reader;
    // Match time (in ms) when the last loop ended
    private double lastLoopEndMs = 0;

    // Last power sent to each drive motor (NaN means nothing has been sent yet)
    private final double[] lastDrivePower = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};

    // Smallest power change worth sending to a motor (0.01 = 1% power)
    private static final double POWER_TOLERANCE = 0.01;

    public void initHardware() {
        robotHardware = new Hardware(hardwareMap);
        intake = robotHardware.robotIntake;
        pinpoint = robotHardware.robotPinpoint;
        controller1 = new GamepadEx(gamepad1);
        lt1Reader = new TriggerReader(controller1, GamepadKeys.Trigger.LEFT_TRIGGER);
        rt1Reader = new TriggerReader(controller1, GamepadKeys.Trigger.RIGHT_TRIGGER);
    }

    public void updateInputs() {
        controller1.readButtons();
        lt1Reader.readValue();
        rt1Reader.readValue();
        pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
    }

    // Sends the four drive powers, skipping any motor whose power barely changed
    protected void setDrivePowers(double fl, double fr, double bl, double br) {
        setIfChanged(robotHardware.frontLeft, 0, fl);
        setIfChanged(robotHardware.frontRight, 1, fr);
        setIfChanged(robotHardware.backLeft, 2, bl);
        setIfChanged(robotHardware.backRight, 3, br);
    }

    // Only talks to the motor if the power changed by at least 1%, or if it's a stop
    private void setIfChanged(DcMotorEx motor, int index, double power) {
        double last = lastDrivePower[index];
        boolean firstTime = Double.isNaN(last);
        boolean stopping = power == 0 && last != 0; // a real 0 always goes through so the motor brakes
        boolean bigChange = Math.abs(power - last) >= POWER_TOLERANCE;
        if (firstTime || stopping || bigChange) {
            motor.setPower(power);
            lastDrivePower[index] = power;
        }
    }
    // Shows how long the last loop took, using the match timer
    protected void addLoopTimeTelemetry() {
        double now = Timer.milliseconds();
        telemetry.addData("Loop (ms)", "%.1f", now - lastLoopEndMs);
        lastLoopEndMs = now;
    }
}