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
    private boolean bToggle = false;
    private Caching_Servo flap;

    private final double FLAP_MIN = 0.36;
    private final double FLAP_MAX = 0.42;

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

    public void setFlap(double pos){
        flap.setPosition(pos);
    }

    public void setFlywheelPower(double value){
        motor.setPower(value);
    }

    public double getFlywheelVelcoity(RevBulkData data){
        return data.getMotorVelocity(motor.motor);
    }

    public void operate(GamepadEx gamepad, RevBulkData data){
        if(Robot.isContinuous() && Intake.toggle){
            startFlywheel();
        }else if(!Robot.isContinuous()){
            if(gamepad.isPress(GamepadEx.Control.a)){
                startFlywheel();
                flicker.resetTime();
                flicker.flick = true;
            }

            if(gamepad.isPress(GamepadEx.Control.b)){
                bToggle = true;
            }

            if(flicker.flick){
                if(bToggle){
                    if(first){
                        flicker.resetTime();
                        first = false;
                    }
                    flicker.flick();
                }
            }else{
                bToggle = false;
                stopFlywheel();
                first = true;
            }
        }

        telemetry.addData("shooter velocity", getFlywheelVelcoity(data));
    }
}
