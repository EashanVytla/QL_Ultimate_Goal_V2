package org.firstinspires.ftc.teamcode.Components;

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
    public static double outPos = 0.281;
    public static double inPos = 0.345;
    public static double flickerSpeed = 0.1;

    Telemetry telemetry;
    Caching_Servo flicker;
    protected boolean flick = false;
    private int flickState = 0;
    private ElapsedTime time;

    protected Flicker(HardwareMap hardwareMap, Telemetry telemetry){
        flicker = new Caching_Servo(hardwareMap, "flicker");
        this.telemetry = telemetry;
        time = new ElapsedTime();
        time.reset();

        if(Robot.isContinuous()){
            setIdlePos(Shooter.ROTATOR_0);
        }else{
            setPos(inPos);
        }
    }

    private double getFlickerPosIdle(double rotationPos){
        double pos = 4.03262 * Math.pow(rotationPos, 2);
        pos += -3.41451 * rotationPos;
        pos += 1.33696;

        telemetry.addData("Rotation Pos", rotationPos);

        if(rotationPos < 0.4){
            return 0.6;
        }else{
            return pos;
        }
    }

    public void setIdlePos(double rotationPos){
        setPos(getFlickerPosIdle(rotationPos));
    }

    public void setPos(double pos){
        flicker.setPosition(pos);
    }

    public void start(){
        time.startTime();
    }

    public void resetTime(){
        time.reset();
    }

    public void flick(){
        if(flickState % 2 == 0){
            if(time.time() > flickerSpeed){
                time.reset();
                flickState++;
            }

            flicker.setPosition(outPos);
        }else if(flickState % 2 == 1){
            flicker.setPosition(inPos);

            if(flickState == 5){
                if(time.time() > flickerSpeed + 1.0){
                    flickState = 0;
                    flick = false;
                }
            }else{
                if(time.time() > flickerSpeed){
                    time.reset();
                    flickState++;
                }
            }
        }
    }

    public void write(){
        flicker.write();
    }
}
