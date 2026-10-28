package org.firstinspires.ftc.teamcode.opmodes;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Hardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

public abstract class BaseTeleOp extends LinearOpMode {
    protected Hardware robotHardware;
    protected Intake intake;
    protected GoBildaPinpointDriver pinpoint;
    protected GamepadEx controller1;
    protected TriggerReader lt1Reader;
    protected TriggerReader rt1Reader;

    public void initHardware() {
        robotHardware = new Hardware(hardwareMap);
        intake = robotHardware.robotIntake;
        pinpoint = robotHardware.pinpoint;
        controller1 = new GamepadEx(gamepad1);
        lt1Reader = new TriggerReader(controller1, GamepadKeys.Trigger.LEFT_TRIGGER);
        rt1Reader = new TriggerReader(controller1, GamepadKeys.Trigger.RIGHT_TRIGGER);
    }

    public void updateInputs() {
        controller1.readButtons();
        lt1Reader.readValue();
        rt1Reader.readValue();
        pinpoint.update();
        }
}
