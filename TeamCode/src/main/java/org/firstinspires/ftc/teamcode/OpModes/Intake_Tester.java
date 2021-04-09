package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp
public class Intake_Tester extends OpMode {
    private Intake intake;
    private GamepadEx gamepadEx;

    @Override
    public void init() {
        intake = new Intake(hardwareMap, telemetry);
        gamepadEx = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        intake.operate(gamepadEx);

        intake.write();
        gamepadEx.loop();
    }
}
