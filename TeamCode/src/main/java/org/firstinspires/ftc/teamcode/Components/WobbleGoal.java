package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class WobbleGoal {
    public final double clamp_posR = 0.8;
    public final double release_posR = 0.185;
    public final double lift_posR = 0.71;
    public final double auto_lift_posR = 0.3;
    public final double down_posR = 0.18;

    public final double clamp_posL = 0.0;
    public final double release_posL = 0.67;
    public final double lift_posL = 0.21;
    public final double auto_lift_posL = 0.52;
    public final double down_posL = 0.73;


    public Caching_Servo servo_liftRight;
    public Caching_Servo servo_grabRight;
    public Caching_Servo servo_grabLeft;
    public Caching_Servo servo_liftLeft;

    private Telemetry telemetry;
    boolean grabberToggle = false;
    boolean grabberLiftToggle = false;
    boolean liftToggle = false;

    public WobbleGoal(HardwareMap map, Telemetry telemetry) {
        servo_liftRight = new Caching_Servo(map, "wobble_liftRight");
        servo_grabRight = new Caching_Servo(map, "wobble_grabRight");
        servo_liftLeft = new Caching_Servo(map, "wobble_liftLeft");
        servo_grabLeft = new Caching_Servo(map, "wobble_grabLeft");

        this.telemetry = telemetry;
    }

    public void init(){
        clamp();
        lift();
        write();
    }

    public void clamp() {
        if(Robot.isBlue()){
            servo_grabLeft.setPosition(clamp_posL);
        }else{
            servo_grabRight.setPosition(clamp_posR);
        }
    }

    public void write() {
        if(Robot.isBlue()){
            servo_grabLeft.write();
            servo_liftLeft.write();
        }else{
            servo_grabRight.write();
            servo_liftRight.write();
        }
    }

    public void release() {
        if(Robot.isBlue()){
            servo_grabLeft.setPosition(release_posL);
        }else{
            servo_grabRight.setPosition(release_posR);
        }
    }

    public void lift() {
        if(Robot.isBlue()){
            servo_liftLeft.setPosition(lift_posL);
        }else{
            servo_liftRight.setPosition(lift_posR);
        }
    }

    public void autoLift() {
        if (Robot.isBlue()) {
            servo_liftLeft.setPosition(auto_lift_posL);
        } else {
            servo_liftRight.setPosition(auto_lift_posR);
        }
    }

    public void down() {
        if(Robot.isBlue()){
            servo_liftLeft.setPosition(down_posL);
        }else{
            servo_liftRight.setPosition(down_posR);
        }
    }

    public void operate(GamepadEx gamepad, GamepadEx gamepad2) {
        if (gamepad.isPress(GamepadEx.Control.right_stick_button)) {
            grabberToggle = !grabberToggle;

            if (grabberToggle) {
                clamp();
            } else {
                release();
            }
        }

        if (gamepad.isPress(GamepadEx.Control.dpad_left)) {
            liftToggle = !liftToggle;
        }

        if (liftToggle) {
            if (gamepad.isPress(GamepadEx.Control.left_stick_button)) {
                if (grabberToggle) {
                    lift();
                } else {
                    down();
                }
                grabberToggle = !grabberToggle;
            }
        } else {
            if(Robot.isBlue()){
                servo_liftLeft.setPosition(0.0);
            }else {
                servo_liftRight.setPosition(0.0);
            }
        }
    }
}
