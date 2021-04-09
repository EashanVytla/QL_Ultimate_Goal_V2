package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class Intake {
    private Caching_Motor[] intake = new Caching_Motor[2];
    public static boolean toggle;
    public boolean inverseToggle;
    private Caching_Motor leveler;

    public Intake(HardwareMap map, Telemetry telemetry){
        toggle = false;
        intake[0] = new Caching_Motor(map, "intake_left");
        intake[1] = new Caching_Motor(map, "intake_right");

        leveler = new Caching_Motor(map, "leveler");

        intake[0].motor.setDirection(DcMotorSimple.Direction.REVERSE);
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
            inverseToggle = false;
        }


        if(gamepad.isPress(GamepadEx.Control.left_bumper)){
            toggle = !toggle;
            setPower(-1.0);
        }else if(toggle){
            setPower(1.0);
        }else{
            setPower(0.0);
        }
    }
}
