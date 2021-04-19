package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

@TeleOp
public class Flicker_Tester extends OpMode {
    Shooter shooter;
    GamepadEx gamepadEx;
    ExpansionHubEx hub;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap, telemetry);
        gamepadEx = new GamepadEx(gamepad1);
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
    }

    public void start(){
        shooter.flicker.start();
    }

    @Override
    public void loop() {
        RevBulkData data = hub.getBulkInputData();

        shooter.write();
        gamepadEx.loop();
    }
}
