package org.firstinspires.ftc.teamcode.Components;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;
import org.openftc.revextensions2.RevBulkData;

public class Shooter {
    private Caching_Motor motor;
    protected Flicker flicker;

    public Shooter(HardwareMap map, Telemetry telemetry){
        motor = new Caching_Motor(map, "shooter");
    }

    public void write(){
        flicker.write();
        motor.write();
    }

    public void startFlywheel(){
        motor.setPower(1.0);
    }

    public void stopFlywheel(){
        motor.setPower(0.0);
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

            if(flicker.flick){
                if(getFlywheelVelcoity(data) > 2200){
                    flicker.flick();
                }
            }else{
                stopFlywheel();
            }
        }
    }
}
