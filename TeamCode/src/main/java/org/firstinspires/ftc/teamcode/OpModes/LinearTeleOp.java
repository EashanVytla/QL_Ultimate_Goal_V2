package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp(name = "TeleOp")
public class LinearTeleOp extends LinearOpMode {
    Robot robot;
    GamepadEx gamepad1ex;
    GamepadEx gamepad2ex;
    private Caching_Servo rotator;
    private Intake intake;
    private final double start_pos_tick = 0.135;
    private final double scale_factor = (130/.95);
    private final double max_tick = 0.3;
    private final double min_tick = 0.0;

    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap, telemetry);
        rotator = new Caching_Servo(hardwareMap, "rotator");
        gamepad1ex = new GamepadEx(gamepad1);
        gamepad2ex = new GamepadEx(gamepad2);
        rotator.setPosition(start_pos_tick);

        waitForStart();

        robot.start();

        while(opModeIsActive()){
            robot.operate(gamepad1ex, gamepad2ex);
            setRotator(robot.getPos().getY(), robot.getPos().getX(), robot.getPos().getHeading());
            robot.converter.tickValue();

            telemetry.addData("Odo", robot.getPos().getY());


            rotator.write();
            telemetry.update();
            gamepad1ex.loop();
            gamepad2ex.loop();
        }
    }

    public void setRotator(double x, double y, double z){
        double targetangle = Math.atan2(robot.ULTIMATE_GOAL_POS.getY() - y, robot.ULTIMATE_GOAL_POS.getX() - x);

        Math.toDegrees(targetangle);

        double currentAngle = Math.toDegrees(z);

        double angle = targetangle - currentAngle;

        double tick_offset = angle / scale_factor;

        double final_position = start_pos_tick + tick_offset;

        if(final_position > max_tick){
            final_position = max_tick;
        } else if(final_position < min_tick){
            final_position = min_tick;
        } else {

        }


       rotator.setPosition(final_position);

        telemetry.addData("Pos", final_position);
        telemetry.addData("Angle", angle);
        telemetry.addData("TargetAngle", Math.toDegrees(targetangle));
        telemetry.addData("CurrentAngle", Math.toDegrees(currentAngle));
    }


}
