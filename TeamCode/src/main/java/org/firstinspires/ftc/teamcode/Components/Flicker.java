package org.firstinspires.ftc.teamcode.Components;

import android.os.SystemClock;
import android.os.health.ServiceHealthStats;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@Config
public class Flicker {
    public static double outPos = 0.23;
    public static double inPos = 0.31;
    public static double flickerSpeed = 70;

    Telemetry telemetry;
    Caching_Servo flicker;
    protected boolean flick = false;
    private int flickState = 0;
    private long prevTime = 0;

    protected Flicker(HardwareMap hardwareMap, Telemetry telemetry){
        flicker = new Caching_Servo(hardwareMap, "flicker");
        this.telemetry = telemetry;

        flicker.setPosition(inPos);
    }

    public void setPos(double pos){
        flicker.setPosition(pos);
    }

    public void reset(){
        flickState = 0;
    }

    public void flickStack(){
        if(flickState % 2 == 0){
            if(getTime() > flickerSpeed){
                resetTime();
                flickState++;
            }

            flicker.setPosition(outPos);
        }else if(flickState % 2 == 1){
            flicker.setPosition(inPos);

            if(flickState == 5){
                if(getTime() > flickerSpeed + 1.0){
                    flickState = 0;
                    flick = false;
                }
            }else{
                if(getTime() > flickerSpeed){
                    resetTime();
                    flickState++;
                }
            }
        }
    }

    private long getTime(){
        return SystemClock.uptimeMillis() - prevTime;
    }

    private void resetTime(){
        prevTime = SystemClock.uptimeMillis();
    }

    public void flick(){
        if(flickState % 2 == 0){
            if(getTime() > flickerSpeed){
                resetTime();
                flickState++;
            }

            flicker.setPosition(outPos);
        }else if(flickState % 2 == 1){
            flicker.setPosition(inPos);

            if(getTime() > flickerSpeed){
                resetTime();
                flickState++;
            }
        }
    }

    public void write(){
        flicker.write();
    }
}
