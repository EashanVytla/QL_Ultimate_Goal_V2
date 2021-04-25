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

    private double barUp = 0.338;
    private double barDown = 0.72;

    private boolean barToggle = false;

    public Intake(HardwareMap map, Telemetry telemetry){
        toggle = false;
        pause = false;
        intake[0] = new Caching_Motor(map, "intake_left");
        intake[1] = new Caching_Motor(map, "intake_right");
        bar = new Caching_Servo(map, "bar");
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
    }

    public void operate(GamepadEx gamepad, GamepadEx gamepad2){
        if(gamepad.isPress(GamepadEx.Control.right_bumper)){
            toggle = !toggle;
        }

        if(gamepad.isPress(GamepadEx.Control.b)){
            barToggle = !barToggle;
        }

        if (barToggle) {
            bar.setPosition((barUp + barDown)/2);
        }else{
            bar.setPosition(barDown);
        }

        if(gamepad.gamepad.left_bumper && !pause){
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
