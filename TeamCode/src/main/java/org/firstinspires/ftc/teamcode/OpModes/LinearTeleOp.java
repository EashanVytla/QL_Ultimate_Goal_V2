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

        while(opModeIsActive() && !isStopRequested()){
            robot.updateBulkData();

            robot.drive.driveCentric(gamepad1, 1.0, 1.0, robot.getPos().getHeading() + Math.toRadians(90));
            robot.flicker.operate(gamepad1ex);

            telemetry.addData("Robot Position:", robot.getPos());

            gamepad1ex.loop();
            gamepad2ex.loop();
            telemetry.update();
            robot.drive.write();
            robot.updatePos();
            robot.flicker.write();
        }
    }
}
