package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class Intake {
    private Caching_Motor[] intake = new Caching_Motor[2];
    public boolean toggle;
    public static boolean isOff = true;
    public static boolean pause = false;
    public Caching_Servo bar;
    public Caching_Servo bar2;

    private double barUp = 0.338;
    private double barDown = 0.69;
    private double barMid = 0.616;

    private boolean barToggle = false;

    public double barClosePos = 0.5;
    private double barOpenPos = 0.85;

    public Intake(HardwareMap map, Telemetry telemetry){
        toggle = false;
        pause = false;
        intake[0] = new Caching_Motor(map, "intake_left");
        intake[1] = new Caching_Motor(map, "intake_right");
        bar = new Caching_Servo(map, "bar");
        bar2 = new Caching_Servo(map, "bar2");
    }

    public void barDown(){
        bar.setPosition(barDown);
    }

    public void barUp(){
        bar.setPosition(barUp);
    }

    public void setPower(double power){
        intake[0].setPower(power);
        intake[1].setPower(power);
    }

    public void write(){
        intake[0].write();
        intake[1].write();
        bar.write();
        bar2.write();
    }

    public void operate(GamepadEx gamepad, GamepadEx gamepad2){
        if(gamepad.isPress(GamepadEx.Control.right_bumper) || gamepad2.isPress(GamepadEx.Control.right_bumper)){
            toggle = !toggle;
        }

        if(gamepad.isPress(GamepadEx.Control.b)){
            barToggle = !barToggle;
        }

        if(gamepad.gamepad.left_trigger > 0.5){
            bar2.setPosition(barClosePos);
        }else{
            bar2.setPosition(barOpenPos);
        }

        if (barToggle) {
            bar.setPosition(barMid);
        }else{
            bar.setPosition(barDown);
        }

        if((gamepad.gamepad.left_bumper || gamepad2.gamepad.left_bumper) && !pause){
            isOff = false;
            setPower(-1.0);
        }else if(toggle && !pause){
            isOff = false;
            setPower(1.0);
        }else{
            isOff = true;
            setPower(0.0);
        }
    }
}
