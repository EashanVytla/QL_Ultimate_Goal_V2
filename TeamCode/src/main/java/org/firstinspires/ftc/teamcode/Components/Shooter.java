package org.firstinspires.ftc.teamcode.Components;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;
import org.openftc.revextensions2.RevBulkData;

@Config
public class Shooter {
    private Caching_Motor motor;
    public Flicker flicker;
    private Telemetry telemetry;
    private boolean first = true;
    private boolean xToggle = false;
    private boolean aToggle = false;
    private Caching_Servo flap;

    private final double FLAP_MIN = 0.36;
    private final double FLAP_MAX = (FLAP_MIN + 0.06);

    private final double ROTATOR_MIN = 0.315;
    private final double ROTATOR_MAX = 0.641;
    private final double ROTATOR_0 = 0.475;
    private final double DEGREES_TO_TICKS = (130/.825);

    private PIDFController flywheelPIDController;

    private Caching_Servo rotator;

    public static double kpF = 0.3;
    public static double kiF = 0.075;
    public static double kdF = 0.015;

    public Shooter(HardwareMap map, Telemetry telemetry){
        motor = new Caching_Motor(map, "shooter");
        flicker = new Flicker(map, telemetry);
        flap = new Caching_Servo(map, "flap");
        rotator = new Caching_Servo(map, "rotator");

        rotator.setPosition(ROTATOR_0);
        flap.setPosition(0.36);

        flywheelPIDController = new PIDFController(new PIDCoefficients(kpF, kiF, kdF));

        motor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.telemetry = telemetry;
    }

    public void write(){
        rotator.write();
        flicker.write();
        motor.write();
        rotator.write();
        flap.write();
    }

    public void startFlywheel(){
        motor.setPower(-1.0);
    }

    public void stopFlywheel(){
        motor.setPower(0.0);
    }

    public double getFlapPos(double distance){
        double a = -5.7241 * Math.pow(10, -11) * Math.pow(distance, 6);
        a += 3.4576 * Math.pow(10, -8) * Math.pow(distance, 5);
        a += -0.0000086196 * Math.pow(distance, 4);
        a += 0.00113449 * Math.pow(distance, 3);
        a += -0.0830887 * Math.pow(distance, 2);
        a += 3.20799 * distance;
        a += -50.5617;

        return a;
    }

    public void setRotator(Pose2d currentPos) {
        double targetangle = Math.atan2((Robot.ULTIMATE_GOAL_POS.getX() - currentPos.getX()), (Robot.ULTIMATE_GOAL_POS.getY() - currentPos.getY()));
        double heading = currentPos.getHeading();

        if(heading <  2 * Math.PI && heading >= Math.PI){
            heading -= 2 * Math.PI;
        }

        double tick_offset = Math.toDegrees(targetangle - heading) / DEGREES_TO_TICKS;

        telemetry.addData("Offset", Math.toDegrees(targetangle - heading));
        telemetry.addData("0 value", ROTATOR_0);

        rotator.setPosition(Range.clip(ROTATOR_0 + tick_offset, ROTATOR_MIN, ROTATOR_MAX));
    }

    public void setFlywheelVelocity(double targetVelo, double currentVelo){
        flywheelPIDController.setTargetPosition(targetVelo);

        setFlywheelPower(flywheelPIDController.update(currentVelo));
    }

    public void setFlywheelPower(double value){
        motor.setPower(-value);
    }

    public double getFlywheelVelcoity(RevBulkData data){
        return data.getMotorVelocity(motor.motor);
    }

    public void operate(GamepadEx gamepadEx, Pose2d currentPos, RevBulkData data, TelemetryPacket packet){
        double flywheelVelo = -data.getMotorVelocity(motor.motor);

        telemetry.addData("Flywheel Velocity", flywheelVelo);
        packet.put("Flywheel Velocity", flywheelVelo);

        if(gamepadEx.isPress(GamepadEx.Control.a)) {
            aToggle = !aToggle;
        }

        if (aToggle) {
            //startFlywheel();
            setFlywheelVelocity(2000, flywheelVelo);

            if (!Robot.isContinuous()) {
                flicker.resetTime();
                flicker.flick = true;

                if (gamepadEx.isPress(GamepadEx.Control.x)) {
                    xToggle = true;
                }

                if (flicker.flick) {
                    if (xToggle) {
                        if (first) {
                            flicker.resetTime();
                            first = false;
                        }
                        flicker.flick();
                    }
                } else {
                    xToggle = false;
                    stopFlywheel();
                    first = true;
                }
            }
        }else{
            setFlywheelPower(0.4);
        }

        flap.setPosition(getFlapPos(Robot.ULTIMATE_GOAL_POS.distTo(currentPos.vec())));
        setRotator(currentPos);

        telemetry.addData("A toggle", aToggle);
        telemetry.addData("distance to goal", Robot.ULTIMATE_GOAL_POS.distTo(currentPos.vec()));
        telemetry.addData("Flap Position", flap.getPosition());


        telemetry.addData("shooter velocity", getFlywheelVelcoity(data));
    }
}