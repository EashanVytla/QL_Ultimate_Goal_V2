package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Flicker;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp(name = "TeleOp")
public class LinearTeleOp extends LinearOpMode {
    Robot robot;
    Shooter shooter;
    GamepadEx gamepad1ex;
    GamepadEx gamepad2ex;

    private ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);

        gamepad1ex = new GamepadEx(gamepad1);
        gamepad2ex = new GamepadEx(gamepad2);
        robot.wobbleGoal.clamp();
        robot.wobbleGoal.write();

        waitForStart();

        time.startTime();

        while(opModeIsActive()){
            robot.updateBulkData();

            telemetry.addData("Start Pos", robot.getStartPos());
            telemetry.addData("Blue? ", Robot.isBlue());

            if(gamepad1ex.isPress(GamepadEx.Control.back)){
                shooter.bigPID = true;
                robot.setStartPose(new Pose2d(0, 0, 0));
                robot.localizer.reset();
            }

            robot.operate(gamepad1ex, gamepad2ex);

            robot.shooter.write();

            telemetry.update();
            gamepad1ex.loop();
            gamepad2ex.loop();
        }
    }
}
