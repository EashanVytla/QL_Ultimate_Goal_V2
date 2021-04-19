package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class Converter {
    private Caching_Motor motor;
    private Telemetry telemetry;
    public boolean bToggle = false;
    public int reset = 0;
    public int change = 1;

    public Converter(HardwareMap map, Telemetry telemetry){
        motor = new Caching_Motor(map,"converter");
        this.telemetry = telemetry;
        motor.setPower(reset);
    }

    public void write(){
        motor.write();
    }

    public void operate(GamepadEx gamepad){
        if(gamepad.isPress(GamepadEx.Control.b)){
            bToggle = !bToggle;
            if(bToggle){
                motor.setPower(change);
            } else {
                motor.setPower(reset);
            }
        }
    }

    public double getTickValue(){
        return motor.getCurrentPosition();
    }
}
