package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake {
    private DcMotorEx flywheel_motor;
    private static final double IN_POWER = 1;
    private static final double OUT_POWER = -1;

    // Takes the motor directly from your Hardware class
    public Intake(DcMotorEx intakeMotor) {
        flywheel_motor = intakeMotor;
    }
    public void in() {
        flywheel_motor.setPower(IN_POWER);
    }

    public void out() {
        flywheel_motor.setPower(OUT_POWER);
    }

    public void stop() {
        flywheel_motor.setPower(0.0);
    }
}
