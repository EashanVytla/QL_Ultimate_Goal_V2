package org.firstinspires.ftc.teamcode.Components;


import android.os.DropBoxManager;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OpModes.LinearTeleOp;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;
import org.openftc.revextensions2.RevBulkData;

@Config
public class Shooter {
    public Caching_Motor motor;
    public Flicker flicker;
    private Telemetry telemetry;
    private boolean xToggle = false;
    private boolean aToggle = false;
    public Caching_Servo flap;

    public static final double FLAP_MIN = 0.725;
    public static final double FLAP_MAX = (FLAP_MIN + 0.06);

    private final double ROTATOR_MIN = 0.25;
    private final double ROTATOR_MAX = 0.641;
    public static final double ROTATOR_0 = 0.475;
    private final double DEGREES_TO_TICKS = (130/.825);

    private PIDFController flywheelPIDController;

    private Caching_Servo rotator;

    public static double kpF = 0.3;
    public static double kiF = 0.075;
    public static double kdF = 0.015;

    public Caching_Servo converter;
    public static double continuousModePos = 0.25;
    public static double flickerModePos = 0.05;

    private double flapTesterPos = 0.5;

    private ElapsedTime time;
    private boolean prevContinuous;

    public Shooter(HardwareMap map, Telemetry telemetry){
        motor = new Caching_Motor(map, "shooter");
        flicker = new Flicker(map, telemetry);
        flap = new Caching_Servo(map, "flap");
        rotator = new Caching_Servo(map, "rotator");
        converter = new Caching_Servo(map, "converter");
        time = new ElapsedTime();
        prevContinuous = Robot.isContinuous();

        rotator.setPosition(ROTATOR_0);
        flap.setPosition(0.2);
        converter.setPosition(continuousModePos);
        write();

        flywheelPIDController = new PIDFController(new PIDCoefficients(kpF, kiF, kdF));

        motor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.telemetry = telemetry;
    }

    public void start(){
        time.startTime();
        flicker.start();
    }

    public void write(){
        rotator.write();
        flicker.write();
        motor.write();
        rotator.write();
        flap.write();
        converter.write();
    }

    public void startFlywheel(){
        motor.setPower(-1.0);
    }

    public void stopFlywheel(){
        motor.setPower(0.0);
    }

    public double getFlapPos(double distance){
        double newDist = Range.clip(distance, 77, 120);
        double a = -7.2889e-10 * Math.pow(newDist, 6);
        a += 4.22626e-7 * Math.pow(newDist, 5);
        a += -0.000101526 * Math.pow(newDist, 4);
        a += 0.0129336 * Math.pow(newDist, 3);
        a += -0.921474 * Math.pow(newDist, 2);
        a += 34.8111 * newDist;
        a += -543.964;

        return Range.clip(a, FLAP_MIN, FLAP_MAX);
    }

    public void resetPID(){
        flywheelPIDController.reset();
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

        double pos = Range.clip(ROTATOR_0 + tick_offset, ROTATOR_MIN, ROTATOR_MAX);

        if(currentPos.getY() >= 84){
            rotator.setPosition(ROTATOR_0);
        }else{
            rotator.setPosition(pos);
        }
    }

    public void setFlap(double pos){
        flap.setPosition(pos);
    }

    public void setRotator(double targetTick) {
        rotator.setPosition(Range.clip(targetTick, ROTATOR_MIN, ROTATOR_MAX));
    }

    public void setFlywheelVelocity(double targetVelo, double currentVelo){
        flywheelPIDController.setTargetPosition(targetVelo);

        setFlywheelPower(flywheelPIDController.update(currentVelo));
    }

    public void setFlywheelPower(double value){
        motor.setPower(-value);
    }

    public double getFlywheelVelcoity(RevBulkData data){
        if(data != null){
            return -data.getMotorVelocity(motor.motor);
        }else{
            return 0;
        }
    }

    private boolean prevIntakeOn = false;
    private boolean switching = false;
    private boolean queryCont = false;
    private boolean first = true;

    public double getRotatorPos(){
        return rotator.getPosition();
    }

    public void operate(GamepadEx gamepad1Ex, GamepadEx gamepad2Ex, Pose2d currentPos, RevBulkData data, TelemetryPacket packet){
        double flywheelVelo = getFlywheelVelcoity(data);

        telemetry.addData("Flywheel Velocity", flywheelVelo);
        packet.put("Flywheel Velocity", flywheelVelo);

        if(currentPos.getY() > 110){
            if(first){
                queryCont = Robot.isContinuous();
                first = false;
            }
            Robot.setContinuous(false);
        }else{
            first = true;
            Robot.setContinuous(queryCont);
        }

        if(gamepad1Ex.isPress(GamepadEx.Control.left_stick_button) || gamepad2Ex.isPress(GamepadEx.Control.a)) {
            aToggle = !aToggle;
        }

        if((Robot.isContinuous() && !prevContinuous) || (!Robot.isContinuous() && prevContinuous)){
            switching = true;
            time.reset();
        }

        if(Robot.isContinuous()){
            rotator.setPosition(0.35);

            if(time.time() > 0.75){
                flicker.setIdlePos(rotator.getPosition());
                converter.setPosition(continuousModePos);
            }else if(time.time() > 0.5){
                flicker.setIdlePos(rotator.getPosition());
                switching = false;
            }else if(time.time() > 0.25){
                converter.setPosition(0.28);
            }
        }else if(!xToggle){
            rotator.setPosition(0.35);

            if(time.time() > 0.5){
                switching = false;
                flicker.setPos(Flicker.inPos);
            }else if(time.time() > 0.25){
                converter.setPosition(flickerModePos);
            }
        }

        telemetry.addData("Converter Position", converter.getPosition());

        if (aToggle) {
            if (!Robot.isContinuous()) {
                startFlywheel();

                if (gamepad1Ex.isPress(GamepadEx.Control.left_trigger)) {
                    Intake.pause = true;
                    flicker.resetTime();
                    flicker.flick = true;
                    xToggle = true;
                }

                if (flicker.flick) {
                    if (xToggle) {
                        flicker.flick();
                    }
                } else {
                    Intake.pause = false;
                    if(xToggle){
                        aToggle = false;
                    }
                    xToggle = false;
                }
            }else{
                setFlywheelVelocity(2000, flywheelVelo);
            }
        }else{
            resetPID();
            if(!LinearTeleOp.powerShots){
                setFlywheelPower(0.0);
            }
        }

        if(prevIntakeOn && Intake.isOff){
            resetPID();
        }

        if(!LinearTeleOp.powerShots && !switching){
            flap.setPosition(getFlapPos(Robot.ULTIMATE_GOAL_POS.distTo(currentPos.vec())));
            setRotator(currentPos);
        }

        //Flicker Regression Tuning
        //_________________________________________________________
        /*if(gamepad2Ex.gamepad.dpad_up){
            flapTesterPos += 0.0001;
        }

        if(gamepad2Ex.gamepad.dpad_down){
            flapTesterPos -= 0.0001;
        }

        flap.setPosition(flapTesterPos);*/
        //----------------------------------------------------------

        telemetry.addData("X toggle", xToggle);
        telemetry.addData("distance to goal", Robot.ULTIMATE_GOAL_POS.distTo(currentPos.vec()));
        telemetry.addData("Flap Position", flap.getPosition());

        prevContinuous = Robot.isContinuous();
        prevIntakeOn = !Intake.isOff;
    }
}