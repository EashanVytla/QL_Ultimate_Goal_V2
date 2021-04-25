package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;
import org.openftc.revextensions2.RevBulkData;

@TeleOp
public class Intake_Tester extends OpMode {
    private Intake intake;
    private Shooter shooter;
    private GamepadEx gamepadEx;
    public static boolean toggle;

    @Override
    public void init() {
        toggle = false;
        intake = new Intake(hardwareMap, telemetry);
        gamepadEx = new GamepadEx(gamepad1);
        shooter = new Shooter(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        intake.operate(gamepadEx, gamepadEx);
        if(gamepadEx.isPress(GamepadEx.Control.y)){
            toggle = !toggle;
        }

        if(toggle){
            shooter.startFlywheel();
        } else {
            shooter.stopFlywheel();
        }

        intake.write();
        shooter.write();
        gamepadEx.loop();
    }
}
