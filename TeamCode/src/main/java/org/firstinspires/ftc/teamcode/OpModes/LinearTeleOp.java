package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp(name = "TeleOp")
public class LinearTeleOp extends LinearOpMode {
    Robot robot;
    GamepadEx gamepad1ex;
    GamepadEx gamepad2ex;
    private Caching_Servo rotator;
    private final double start_pos_tick = 0.341;
    private final double ticks_to_degrees = (0.95/130);
    private final double max_tick = 0.59;
    private final double min_tick = 0.17;

    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap, telemetry);
        rotator = new Caching_Servo(hardwareMap, "rotator");
        gamepad1ex = new GamepadEx(gamepad1);
        gamepad2ex = new GamepadEx(gamepad2);
        rotator.setPosition(0.5);

        waitForStart();

        robot.start();

        while(opModeIsActive() && !isStopRequested()){
            robot.operate(gamepad1ex, gamepad2ex);
            setRotator(robot.getPos().getX(), robot.getPos().getY());
            robot.converter.tickValue();

            rotator.write();
            telemetry.update();
        }
    }

    public void setRotator(double x, double y){
        double angle = Math.atan2(robot.ULTIMATE_GOAL_POS.getY() - y, robot.ULTIMATE_GOAL_POS.getX() - x);

        double pos = angle * ticks_to_degrees;

        if(pos > max_tick){
            pos = max_tick;
        } else if(pos < min_tick){
            pos = min_tick;
        } else {

        }

        rotator.setPosition(start_pos_tick + pos);
        telemetry.addData("Pos", rotator.getPosition());
    }
}
