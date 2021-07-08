package org.firstinspires.ftc.teamcode.OpModes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Components.Robot;

@TeleOp
public class Ma3_Tester extends OpMode {
    AnalogInput encoder;
    double MAX_VOLTAGE = 3.263;

    public void init(){
        encoder = hardwareMap.get(AnalogInput.class, "MA3");
    }

    public void loop(){
        double angle = (encoder.getVoltage()/MAX_VOLTAGE) * (2 * Math.PI);
        telemetry.addData("Encoder Data", Math.toDegrees(angle));
    }
}
