package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake {
    private DcMotorEx motor;
    private static final double IN_POWER = 1;
    private static final double OUT_POWER = -1;

    // Takes the motor directly from your Hardware class
    public Intake(DcMotorEx intakeMotor) {
        motor = intakeMotor;
    }
    public void in() {
        motor.setPower(IN_POWER);
    }

    public void out() {
        motor.setPower(OUT_POWER);
    }

    public void stop() {
        motor.setPower(0.0);
    }
}
