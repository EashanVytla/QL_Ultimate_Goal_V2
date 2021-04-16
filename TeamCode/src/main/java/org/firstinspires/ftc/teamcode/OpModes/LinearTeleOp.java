package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp(name = "TeleOp")
public class LinearTeleOp extends LinearOpMode {
    Robot robot;
    GamepadEx gamepad1ex;
    GamepadEx gamepad2ex;

    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap, telemetry);
        gamepad1ex = new GamepadEx(gamepad1);
        gamepad2ex = new GamepadEx(gamepad2);

        waitForStart();

        robot.start();

        while(opModeIsActive() && !isStopRequested()){
            robot.operate(gamepad1ex, gamepad2ex);

            telemetry.addData("Y button", gamepad1ex.isPress(GamepadEx.Control.y));

            telemetry.addData("Y Power", gamepad1ex.gamepad.left_stick_x);

            telemetry.update();
        }
    }
}
