package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

class Intake {
    private Caching_Motor intake;
    public static boolean toggle;

    public Intake(HardwareMap map, Telemetry telemetry){
        toggle = false;
        intake = new Caching_Motor(map, "intake");
    }

    public void start(){
        intake.setPower(1.0);
    }

    public void stop(){
        intake.setPower(0.0);
    }

    public void setPower(double power){
        intake.setPower(power);
    }

    public void write(){
        intake.write();
    }

    public void operate(GamepadEx gamepad){
        if(gamepad.isPress(GamepadEx.Control.right_bumper)){
            toggle = !toggle;
        }

        if(toggle){
            start();
        }else{
            stop();
        }
    }
}
