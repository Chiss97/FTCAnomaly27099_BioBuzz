    package org.firstinspires.ftc.teamcode.opmodes;

    import com.arcrobotics.ftclib.gamepad.GamepadKeys;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

    import org.firstinspires.ftc.teamcode.util.DriveInputFilter;
    import org.firstinspires.ftc.teamcode.util.Timer;


    @TeleOp(name = "RobotCentric TeleOp", group = "TeleOp")
    public class RobotCentric extends BaseTeleOp{
        private final DriveInputFilter smoother = new DriveInputFilter();
        @Override
        public void runOpMode() throws InterruptedException {

            initHardware();

            // update telemetry to show INITIALIZED status
            telemetry.addData("Status", "INITIALIZED");
            telemetry.update();

            // wait for user to press start button
            waitForStart();
            Timer.start();
            smoother.reset();
            // start OpMode loop
            while (opModeIsActive()) {

                updateInputs();

                //<editor-fold desc="Drivetrain">
                double ly = smoother.shape(controller1.getLeftY());
                double lx = smoother.shape(controller1.getLeftX());
                double rx = smoother.shape(controller1.getRightX());

                smoother.update(ly, lx, rx);
                ly = smoother.out[0];
                lx = smoother.out[1];
                rx = smoother.out[2];

                // Calculate motor powers
                double frontLeftPower = (ly + lx + rx);
                double frontRightPower = (ly - lx - rx);
                double backLeftPower = (ly - lx + rx);
                double backRightPower = (ly + lx - rx);

                // scale down if any motor is over 1.0
                double maxPower = Math.max(1.0, Math.max(
                        Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

                setDrivePowers(frontLeftPower / maxPower, frontRightPower / maxPower,
                        backLeftPower / maxPower, backRightPower / maxPower);
                //</editor-fold>

                if (lt1Reader.isDown()) {
                    intake.in();
                }
                else if(controller1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                    intake.out();
                }
                else{
                    intake.stop();
                }

                //<editor-fold desc="Telemetry">
                addLoopTimeTelemetry();
                telemetry.update();
                //</editor-fold>
            }

        }

    }
