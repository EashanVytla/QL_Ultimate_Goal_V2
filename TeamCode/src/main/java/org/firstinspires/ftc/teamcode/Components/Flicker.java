package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class Flicker {
    private final double outPos = 0.48;
    private final double inPos = 0.55;
    private final double flickerSpeed = 0.04;

    Telemetry telemetry;
    Caching_Servo flicker;
    private boolean flick = false;
    private int flickState = 0;
    private ElapsedTime time;

    protected Flicker(HardwareMap hardwareMap, Telemetry telemetry){
        flicker = new Caching_Servo(hardwareMap, "flicker");

        flicker.setPosition(inPos);
        flicker.write();

        this.telemetry = telemetry;
        time = new ElapsedTime();
    }

    public void start(){
        time.startTime();
    }

    public void operate(GamepadEx gamepad1){
        if(gamepad1.isPress(GamepadEx.Control.a)){
            time.reset();
            flick = true;
        }

        if(flick){
            if(flickState % 2 == 0){
                if(time.time() > flickerSpeed){
                    time.reset();
                    flickState++;
                }

                flicker.setPosition(outPos);
            }else if(flickState % 2 == 1){
                if(time.time() > flickerSpeed){
                    time.reset();
                    flickState++;
                }

                flicker.setPosition(inPos);

                if(flickState == 5){
                    flickState = 0;
                    flick = false;
                }
            }
        }
    }

    public void write(){
        flicker.write();
    }
}
