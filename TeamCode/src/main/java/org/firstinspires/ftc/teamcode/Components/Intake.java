package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class Intake {
    private Caching_Motor[] intake = new Caching_Motor[2];
    private final double leftDropDownClose = 0.85;
    private final double leftDropDownRelease = 0.98;
    private final double rightDropDownClose = 0.1375;
    private final double rightDropDownRelease = 0.0;

    private Caching_Servo dropDownL;
    private Caching_Servo dropDownR;

    public boolean toggle;
    public static boolean isOff = true;
    public static boolean pause = false;

    public Intake(HardwareMap map, Telemetry telemetry){
        toggle = false;
        pause = false;
        intake[0] = new Caching_Motor(map, "intake_left");
        intake[1] = new Caching_Motor(map, "intake_right");

        dropDownL = new Caching_Servo(map, "dropdownL");
        dropDownR = new Caching_Servo(map, "dropdownR");
    }

    public void initAuto(){
        dropDownL.setPosition(leftDropDownClose);
        dropDownR.setPosition(rightDropDownClose);
    }

    public void release(){
        dropDownL.setPosition(leftDropDownRelease);
        dropDownR.setPosition(rightDropDownRelease);
    }

    public void setPower(double power){
        intake[0].setPower(power);
        intake[1].setPower(power);
    }

    public void write(){
        dropDownL.write();
        dropDownR.write();
        intake[0].write();
        intake[1].write();
    }

    public void operate(GamepadEx gamepad, GamepadEx gamepad2){
        if(gamepad.isPress(GamepadEx.Control.right_bumper) || gamepad2.isPress(GamepadEx.Control.right_bumper)){
            toggle = !toggle;
        }

        if((gamepad.gamepad.left_bumper || gamepad2.gamepad.left_bumper) && !pause){
            isOff = false;
            setPower(-1.0);
        }else if(toggle && !pause){
            isOff = false;
            setPower(1.0);
        }else{
            isOff = true;
            setPower(0.0);
        }
    }
}
