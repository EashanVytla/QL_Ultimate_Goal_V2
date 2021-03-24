package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

public class S4T_Encoder {
    ExpansionHubMotor encoder;
    public float distance;
    boolean reverse = false;

    public S4T_Encoder(HardwareMap hardwareMap, String name){
        encoder = (ExpansionHubMotor) hardwareMap.get(DcMotor.class, name);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void reset(){
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void update(RevBulkData data){
        distance = data.getMotorCurrentPosition(encoder);
    }

    public double getDist(){
        if(reverse){
            return -(distance/1440) * ((58/25.4) * Math.PI);
        }else{
            return (distance/1440) * ((58/25.4) * Math.PI);
        }
    }
}
