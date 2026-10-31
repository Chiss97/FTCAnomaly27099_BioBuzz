    package org.firstinspires.ftc.teamcode.robot.subsystems;

    import com.qualcomm.robotcore.hardware.DcMotorEx;

    public class Intake {
        private DcMotorEx motor;
        private static final double IN_POWER = 0.5;
        private static final double OUT_POWER = -0.5;

        private double lastPower = 0;

        // Takes the motor directly from your Hardware class
        public Intake(DcMotorEx intakeMotor) {
            motor = intakeMotor;
        }

        private void setPower(double power) {
            if (power != lastPower) { // Only send when the power changes, for efficiency
                motor.setPower(power);
                lastPower = power;
            }
        }

        public void in() {
            setPower(IN_POWER);
        }

        public void out() {
            setPower(OUT_POWER);
        }

        public void stop() {
            setPower(0.0);
        }
    }
