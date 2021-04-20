package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class Intake {
    private Caching_Motor[] intake = new Caching_Motor[2];
    public boolean toggle;
    public static boolean isOff = true;

    public Intake(HardwareMap map, Telemetry telemetry){
        toggle = false;
        intake[0] = new Caching_Motor(map, "intake_left");
        intake[1] = new Caching_Motor(map, "intake_right");
    }

    public void setPower(double power){
        intake[0].setPower(power);
        intake[1].setPower(power);
    }

    public void write(){
        intake[0].write();
        intake[1].write();
    }


    public void operate(GamepadEx gamepad){
        if(gamepad.isPress(GamepadEx.Control.right_bumper)){
            toggle = !toggle;
        }

        if(gamepad.gamepad.left_bumper){
            isOff = false;
            setPower(-1.0);
        }else if(toggle){
            isOff = false;
            setPower(1.0);
        }else{
            isOff = true;
            setPower(0.0);
        }
    }
}
