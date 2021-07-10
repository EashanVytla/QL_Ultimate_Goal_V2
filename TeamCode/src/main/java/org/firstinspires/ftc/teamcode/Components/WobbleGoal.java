package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class WobbleGoal {
    public final double clamp_pos = 0;
    public final double grabber_idle = 0.489;
    public final double release_pos = 0.8385;
    public final double lift_pos = 0.59;
    public final double drop_over_lift_pos = 0.223;
    public final double auto_lift_pos = 0.552;
    public final double down_pos = 0.18;

    public Caching_Servo servo_lift;
    public Caching_Servo servo_grab;

    private Telemetry telemetry;
    boolean grabberToggle = false;
    int grabberLiftToggle = 0;
    boolean liftToggle = false;

    public WobbleGoal(HardwareMap map, Telemetry telemetry){
        servo_lift = new Caching_Servo(map, "wobble_lift");
        servo_grab = new Caching_Servo(map, "wobble_grab");

        this.telemetry = telemetry;
    }

    public void init(){
        servo_grab.setPosition(release_pos);
        servo_lift.setPosition(lift_pos);
        servo_lift.write();
        servo_grab.write();
    }

    public void clamp(){
        servo_grab.setPosition(clamp_pos);
    }

    public void kick() {
        servo_grab.setPosition(release_pos);
    }

    public void write(){
        servo_grab.write();
        servo_lift.write();
    }

    public void release(){
        servo_grab.setPosition(release_pos);
    }

    public void lift(){
        servo_lift.setPosition(lift_pos);
    }

    public void autoLift(){
        servo_lift.setPosition(auto_lift_pos);
    }

    public void down(){
        servo_lift.setPosition(down_pos);
    }

    public void DropOverWallLift(){
        servo_lift.setPosition(drop_over_lift_pos);
    }

    public void operate(GamepadEx gamepad, GamepadEx gamepad2){
        if(gamepad.isPress(GamepadEx.Control.right_stick_button)){
            grabberToggle = !grabberToggle;

            if(grabberToggle){
                clamp();
            }else{
                kick();
            }
        }

        if(gamepad2.isPress(GamepadEx.Control.x)){
           liftToggle = !liftToggle;
        }

        if(liftToggle){
            if(gamepad2.isPress(GamepadEx.Control.y)){
                if(grabberLiftToggle == 0){
                    down();
                }/*else if(grabberLiftToggle == 1){
                midLift();
            }*/else if(grabberLiftToggle == 1){
                    lift();
                }else if(grabberLiftToggle == 2){
                    DropOverWallLift();
                }

                grabberLiftToggle = (grabberLiftToggle + 1) % 3;
            }
        }else{
            servo_lift.setPosition(0.0);
        }
    }
}
