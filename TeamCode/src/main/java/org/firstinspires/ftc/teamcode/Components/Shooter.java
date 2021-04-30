package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    public Caching_Servo stopper;

    public static double flywheelTargetVelo = 2000;

    public static final double FLAP_MIN = 0.725;
    public static final double FLAP_MAX = 0.78;

    private final double ROTATOR_MIN = 0.25;
    private final double ROTATOR_MAX = 0.641;
    public static final double ROTATOR_0 = 0.475;
    private final double DEGREES_TO_TICKS = (130/.825);

    private PIDFController flywheelPIDController;

    private Caching_Servo rotator;

    public static double kpF = 0.01;
    public static double kiF = 0;
    public static double kdF = 0.0001;

    public static double kS = 0.0;
    public static double kV = 1.075;

    public Caching_Servo converter;
    public static double continuousModePos = 0.30;
    public static double transitionModePos = 0.40;
    public static double flickerModePos = 0.125;

    private double flapTesterPos = 0.7;

    public double stopperIn = 0.92;
    public double stopperOpen = 0.35;

    private ElapsedTime time;
    private boolean prevContinuous;
    private boolean autoAllign = false;
    private boolean manualFlickerToggle = false;

    SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(kS, kV);

    public Shooter(HardwareMap map, Telemetry telemetry){
        motor = new Caching_Motor(map, "shooter");
        flicker = new Flicker(map, telemetry);
        flap = new Caching_Servo(map, "flap");
        rotator = new Caching_Servo(map, "rotator");
        converter = new Caching_Servo(map, "converter");
        stopper = new Caching_Servo(map, "stopper");
        stopper.setPosition(stopperIn);
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
        stopper.write();
    }

    public void startFlywheel(){
        motor.setPower(-1.0);
    }

    public void stopFlywheel(){
        motor.setPower(0.0);
    }

    public double getFlapPos(double distance){
        double newDist = Range.clip(distance, 75, 120);
        /*double a = 3.02204665e-5 * Math.pow(newDist, 1);
        a += 1.16177569e-3 * Math.pow(newDist, 2);
        a += -3.26492093e-5 * Math.pow(newDist, 3);
        a += 3.84664299e-7 * Math.pow(newDist, 4);
        a += -2.14517587e-9 * Math.pow(newDist, 5);
        a += 4.67677460e-12 * Math.pow(newDist, 6);
        a += 0.07692692731456108;*/

        double[] arr = new double[]{-6.69173811e-04, -2.41404719e-02,  7.05637256e-04,
                -8.65084574e-06,  5.00081670e-08, -1.12346487e-10};

        double a = 13.949544261905274;

        for(int i = 0; i < arr.length; i++){
            a += arr[i] * Math.pow(newDist, i + 1);
        }

        telemetry.addData("Regressions Position", a);

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

        telemetry.addData("feedforward", feedforward.calculate(targetVelo)/2800);

        setFlywheelPower(flywheelPIDController.update(currentVelo) + (feedforward.calculate(targetVelo)/2800));
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

    //private boolean prevIntakeOn = false;
    private boolean switching = false;
    private boolean stopperToggle = false;

    public double getRotatorPos(){
        return rotator.getPosition();
    }

    public void operate(GamepadEx gamepad1Ex, GamepadEx gamepad2Ex, Pose2d currentPos, RevBulkData data, TelemetryPacket packet){
        double flywheelVelo = getFlywheelVelcoity(data);

        telemetry.addData("Flywheel Velocity", flywheelVelo);
        packet.put("Flywheel Velocity", flywheelVelo);
        telemetry.addData("Dist to Ultimate Goal", currentPos.vec().distTo(Robot.ULTIMATE_GOAL_POS));

        if(gamepad2Ex.isPress(GamepadEx.Control.b)){
            stopperToggle = !stopperToggle;
            if(stopperToggle){
                stopper.setPosition(stopperOpen);
            }else{
                stopper.setPosition(stopperIn);
            }
        }


        if(gamepad1Ex.isPress(GamepadEx.Control.left_stick_button) || gamepad2Ex.isPress(GamepadEx.Control.a)) {
            aToggle = !aToggle;
        }

        if(gamepad2Ex.isPress(GamepadEx.Control.dpad_right)){
            autoAllign = !autoAllign;
        }

        if((Robot.isContinuous() && !prevContinuous)){
            autoAllign = true;
            switching = true;
            time.reset();
        }

        if((!Robot.isContinuous() && prevContinuous)){
            switching = true;
            time.reset();
        }

        if(Robot.isContinuous()){
            if(time.time() > 0.75){
                flicker.setIdlePos(rotator.getPosition());
                converter.setPosition(continuousModePos);
            }else if(time.time() > 0.5){
                flicker.setIdlePos(rotator.getPosition());
                switching = false;
            }else if(time.time() > 0.25){
                rotator.setPosition(0.35);
                converter.setPosition(transitionModePos);
            }
        }else if(!xToggle){
            if(time.time() > 0.5){
                switching = false;
                flicker.setPos(Flicker.inPos);
            }else if(time.time() > 0.25){
                rotator.setPosition(0.35);
                converter.setPosition(flickerModePos);
            }
        }

        if (aToggle) {
            if (!Robot.isContinuous()) {
                //setFlywheelPower(1.0);
                setFlywheelVelocity(flywheelTargetVelo, flywheelVelo);

                if (gamepad2Ex.isPress(GamepadEx.Control.left_trigger)) {
                    Intake.pause = true;
                    flicker.reset();
                    flicker.flick = true;
                    xToggle = true;
                }

                if(gamepad1Ex.isPress(GamepadEx.Control.right_trigger) || gamepad2Ex.isPress(GamepadEx.Control.right_trigger)){
                    flicker.reset();
                }

                if(gamepad1Ex.gamepad.right_trigger > 0.5 || gamepad2Ex.gamepad.right_trigger > 0.5){
                    manualFlickerToggle = true;
                }else{
                    manualFlickerToggle = false;
                }

                if(manualFlickerToggle){
                    flicker.flick();
                }

                if (flicker.flick) {
                    if (xToggle) {
                        flicker.flickStack();
                    }
                } else {
                    Intake.pause = false;
                    if(xToggle){
                        aToggle = false;
                    }
                    xToggle = false;
                }
            }else{
                //startFlywheel();
                setFlywheelVelocity(flywheelTargetVelo, flywheelVelo);
            }
        }else{
            Intake.pause = false;
            //resetPID();
            stopFlywheel();
        }

        /*if(prevIntakeOn && Intake.isOff){
            resetPID();
        }*/

        if(!switching){
            flap.setPosition(getFlapPos(Robot.ULTIMATE_GOAL_POS.distTo(currentPos.vec())), 1e-6);
            if(autoAllign){
                setRotator(currentPos);
            }else{
                setRotator(ROTATOR_0);
            }
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

        telemetry.addData("Flap Position", flap.getPosition());
        prevContinuous = Robot.isContinuous();
        //prevIntakeOn = !Intake.isOff;
    }
}