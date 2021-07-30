package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Flicker;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp(name = "Blue TeleOp")
public class BlueTeleOp extends LinearOpMode {
    Robot robot;
    Shooter shooter;
    GamepadEx gamepad1ex;
    GamepadEx gamepad2ex;

    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);

        robot.setStartPose(new Pose2d(13.5,0,0));

        robot.blue();

        gamepad1ex = new GamepadEx(gamepad1);
        gamepad2ex = new GamepadEx(gamepad2);

        waitForStart();

        while(opModeIsActive()){
            robot.updateBulkData();

            if(gamepad1ex.isPress(GamepadEx.Control.back)){
                robot.localizer.reset();
                robot.setStartPose(new Pose2d(0, 0, 0));
            }

            robot.operate(gamepad1ex, gamepad2ex);

            telemetry.update();
            gamepad1ex.loop();
            gamepad2ex.loop();
        }
    }
}
