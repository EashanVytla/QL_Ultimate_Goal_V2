package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.RevBulkData;

public class Ma3_Encoder {
    AnalogInput encoder;
    double MAX_VOLTAGE = 3.263;

    public Ma3_Encoder(HardwareMap map, String name){
        encoder = map.get(AnalogInput.class, name);
    }

    public double getRadians(){
        return (encoder.getVoltage()/MAX_VOLTAGE) * (2 * Math.PI);
    }

    public double getRawValue(){
        return encoder.getVoltage()/MAX_VOLTAGE;
    }

    public double getRadians(double offset){
        double angle = ((encoder.getVoltage()/MAX_VOLTAGE) * (2 * Math.PI)) + offset;

        //243.5
        if(angle < 0){
            return (2 * Math.PI) + (angle % (2 * Math.PI));
        }else{
            return angle % (2 * Math.PI);
        }
    }

    public double getRadians(double offset, RevBulkData data){
        double angle = ((data.getAnalogInputValue(encoder)/MAX_VOLTAGE) * (2 * Math.PI)) + offset;

        //243.5
        if(angle < 0){
            return (2 * Math.PI) + (angle % (2 * Math.PI));
        }else{
            return angle % (2 * Math.PI);
        }
    }
}
