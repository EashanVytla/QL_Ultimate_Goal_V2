package org.firstinspires.ftc.teamcode.Wrapper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.RevBulkData;

public class Caching_Motor {
    HardwareMap hardwareMap;
    String name;
    public DcMotorEx motor;
    double prev_power = 0.0;

    double query = -2.0;

    float pos = 0;

    double EPSILON = 0.01;

    double prev_write = 0;
    double current_write = 0;

    public Caching_Motor(HardwareMap hardwareMap, String name){
        this.hardwareMap = hardwareMap;
        this.name = name;

        motor = hardwareMap.get(DcMotorEx.class, name);
    }

    public void setPower(double power){
        if (Math.abs(prev_power - power) > EPSILON){
            query = power;
        }
        else{
            query = -2.0;
        }
    }

    public void reset(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void write(){
        if (query != -2.0) {
            motor.setPower(query);
            prev_power = query;
        }
    }

    public float getCurrentPosition(){
        return pos;
    }

    public void read(RevBulkData data){
        pos = data.getMotorCurrentPosition(motor);
    }
}