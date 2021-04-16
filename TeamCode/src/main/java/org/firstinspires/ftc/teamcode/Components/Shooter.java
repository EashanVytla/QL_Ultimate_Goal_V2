package org.firstinspires.ftc.teamcode.Components;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;
import org.openftc.revextensions2.RevBulkData;

import kotlin.text.CharDirectionality;

public class Shooter {
    private Caching_Motor motor;
    public Flicker flicker;
    private Telemetry telemetry;
    private boolean first = true;
    private boolean xToggle = false;
    private boolean aToggle = false;
    private Caching_Servo flap;

    private final double FLAP_MIN = 0.36;
    private final double FLAP_MAX = (FLAP_MIN + 0.06);

    public Shooter(HardwareMap map, Telemetry telemetry){
        motor = new Caching_Motor(map, "shooter");
        flicker = new Flicker(map, telemetry);
        flap = new Caching_Servo(map, "flap");
        this.telemetry = telemetry;
    }

    public void write(){
        flicker.write();
        motor.write();
        flap.write();
    }

    public void startFlywheel(){
        motor.setPower(-1.0);
    }

    public void stopFlywheel(){
        motor.setPower(0.0);
    }

    public void setFlap(double distance){
        double function;

        flap.setPosition(distance);
    }


    public void setFlywheelPower(double value){
        motor.setPower(value);
    }

    public double getFlywheelVelcoity(RevBulkData data){
        return data.getMotorVelocity(motor.motor);
    }

    public void operate(GamepadEx gamepad, RevBulkData data){
        if(gamepad.isPress(GamepadEx.Control.a)) {
            aToggle = !aToggle;

            if (aToggle) {
                startFlywheel();
                flicker.resetTime();
                flicker.flick = true;

                if (gamepad.isPress(GamepadEx.Control.x)) {
                    xToggle = true;
                }

                if (flicker.flick) {
                    if (xToggle) {
                        if (first) {
                            flicker.resetTime();
                            first = false;
                        }
                        flicker.flick();
                    }
                } else {
                    xToggle = false;
                    stopFlywheel();
                    first = true;
                }
            }
        } else {
            setFlywheelPower(-0.4);
        }

        telemetry.addData("A toggle", aToggle);

        telemetry.addData("shooter velocity", getFlywheelVelcoity(data));
    }
}