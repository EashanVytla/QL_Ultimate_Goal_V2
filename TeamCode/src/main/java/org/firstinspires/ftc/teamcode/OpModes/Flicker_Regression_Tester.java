package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp
public class Flicker_Regression_Tester extends LinearOpMode {
    //Set the hardware mapping name of the servo
    final String name = "flicker";
    final String name2 = "rotator";

    Servo servo;
    Servo servo2;
    private double pos;
    GamepadEx gamepadEx;
    private boolean servoToPosToggle = false;

    Robot robot;
    private double pos2;

    @Override
    public void runOpMode(){
        gamepadEx = new GamepadEx(gamepad1);
        servo = hardwareMap.servo.get(name);
        servo2 = hardwareMap.servo.get(name2);
        pos = FlickerRegressionTester.pos;
        pos2 = FlickerRegressionTester.pos2;
        robot = new Robot(hardwareMap, telemetry);

        waitForStart();
        while(opModeIsActive()) {
            robot.updateBulkData();

            if(gamepadEx.isPress(GamepadEx.Control.a)){
                servoToPosToggle = !servoToPosToggle;
            }

            if(servoToPosToggle){
                telemetry.addData("Mode", "In set position mode...");
                telemetry.addData("    ", "You can tune this position through dashboard.");
                servo.setPosition(ServoTester.pos);
            }else{
                if (gamepad1.dpad_up) {
                    if(pos < 1){
                        pos += 0.0005;
                    }
                } else if (gamepad1.dpad_down){
                    if(pos > 0){
                        pos -= 0.0005;
                    }
                }

                if (gamepad1.dpad_left) {
                    if(pos2 < 1){
                        pos2 += 0.0005;
                    }
                } else if (gamepad1.dpad_right){
                    if(pos2 > 0){
                        pos2 -= 0.0005;
                    }
                }

                telemetry.addData("Mode", "In dynamic position mode..");
                telemetry.addData("    ", "Use the Dpads to change the position dynamically");
                telemetry.addData("    ", "Press A again to go back into set position mode");
                servo.setPosition(pos);
                servo2.setPosition(pos2);
            }

            telemetry.addData("Flicker Position", servo.getPosition());
            telemetry.addData("Rotator Position", servo2.getPosition());

            telemetry.update();
            gamepadEx.loop();
        }
    }
}

@Config
class FlickerRegressionTester{
    //Set the set/start position of the servo in dashboard
    public static double pos = 0.345;
    public static double pos2 = 0.475;
}
