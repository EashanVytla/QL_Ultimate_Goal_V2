package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp(name = "TeleOp")
public class LinearTeleOp extends LinearOpMode {
    Robot robot;
    Shooter shooter;
    GamepadEx gamepad1ex;
    GamepadEx gamepad2ex;


    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);


        robot.localizer.reset();
        gamepad1ex = new GamepadEx(gamepad1);
        gamepad2ex = new GamepadEx(gamepad2);

        waitForStart();

        robot.start();

        while(opModeIsActive()){
            robot.operate(gamepad1ex, gamepad2ex);

            telemetry.update();
            gamepad1ex.loop();
            gamepad2ex.loop();
        }
    }
}
