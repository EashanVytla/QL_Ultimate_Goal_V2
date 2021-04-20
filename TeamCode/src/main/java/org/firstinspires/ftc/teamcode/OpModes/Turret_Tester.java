package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp
public class Turret_Tester extends LinearOpMode {
    //Set the hardware mapping name of the servo
    final String name = "rotator";

    Servo servo;
    private double pos;
    GamepadEx gamepadEx;
    private boolean servoToPosToggle = false;

    Robot robot;

    @Override
    public void runOpMode(){
        gamepadEx = new GamepadEx(gamepad1);
        servo = hardwareMap.servo.get(name);
        pos = ServoTester.pos;
        robot = new Robot(hardwareMap, telemetry);

        waitForStart();
        while(opModeIsActive()) {
            robot.updateBulkData();

            if(gamepadEx.isPress(GamepadEx.Control.a)){
                servoToPosToggle = !servoToPosToggle;
            }

            double flywheelVelo = robot.shooter.getFlywheelVelcoity(robot.getData2());

            robot.shooter.setFlap(TurretTester.flapPos);
            robot.shooter.setFlywheelVelocity(1750, flywheelVelo);
            telemetry.addData("Flywheel Velocity", flywheelVelo);

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

                telemetry.addData("Mode", "In dynamic position mode..");
                telemetry.addData("    ", "Use the Dpads to change the position dynamically");
                telemetry.addData("    ", "Press A again to go back into set position mode");
                servo.setPosition(pos);
            }

            telemetry.addData("Position", servo.getPosition());
            telemetry.update();
            gamepadEx.loop();
            robot.shooter.motor.write();
            robot.shooter.flap.write();
        }
    }
}

@Config
class TurretTester{
    //Set the set/start position of the servo in dashboard
    public static double pos = 0.475;
    public static double flapPos = 0.411;
}
