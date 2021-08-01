package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.ejml.dense.block.VectorOps_DDRB;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;
import org.openftc.revextensions2.RevBulkData;

import java.time.OffsetDateTime;

@Config
public class Shooter {
    public Caching_Motor flywheelMotor; // Flywheel Motor
    public Flicker flicker; // Servo to flick the rings into the flywheel
    private Telemetry telemetry; // Telemetry instance to output data onto the phone
    private boolean flywheelToggle = false; // Toggle the flywheel on and off
    public Caching_Servo flap; // Flap servo to change the height of the ring
    public Caching_Servo stopper; // Servo that stops the ring from falling out of the bucket

    public static double flywheelTargetVelo = 2000; // Target velocity for the flywheel

    // The physical servo constraints for the flap
    public static final double FLAP_MIN = 0.18;
    public static final double FLAP_MAX = 0.93;

    private final double DEGREES_TO_TICKS = (130/.825);
    public static double flapTesterPos = 0.75;

    private final PIDFController flywheelPIDController; // PID controller for the Flywheel
    private final SimpleMotorFeedforward feedForward; // Flywheel feed-forward controller

    public final Caching_Motor rotator; //Bucket rotation motor
    private final PIDFController rotatorPIDController; // PID controller for the Rotator
    private final PIDFController rotatorPIDController2; // PID controller for the Rotator Larger Targets
    private final double ROTATOR_MIN;
    private final double ROTATOR_0; //Rotator 0
    private final double ROTATOR_MAX; //Rotator Max Angle

    //PID constants for the flywheel (static for dashboard live tuning purposes)
    public static double kpF = 0.01;
    public static double kiF = 0;
    public static double kdF = 0.0001;

    //PID constants for the Bucket Rotator (static for dashboard live tuning purposes)
    public static double kpR = 4.75;
    public static double kiR = 4.75;
    public static double kdR = 0.006;

    //PID constants for Larger Distances on the Bucket Rotator (static for dashboard live tuning purposes)
    public static double kpR_2 = 3.5;
    public static double kiR_2 = 0.55;
    public static double kdR_2 = 0.15;

    //Feed-forward constants for the flywheel (static for dashboard live tuning purposes)
    public static double kS = 0.0;
    public static double kV = 1.075;

    public double stopperIn = 0.63;
    public double stopperOpen = 0.0;
    private boolean stopperToggle = false;

    private int powerShotToggle = 0;

    public boolean bigPID = true;
    private double flapOffset = 0.0;
    private double rotatorOffset = 0.0;
    private boolean flapToggle = false;
    //private boolean midGoalToggle = false;

    private ElapsedTime timer = new ElapsedTime();
    private RevBulkData data2;
    private boolean speedToggle = false;

    public Shooter(HardwareMap map, Telemetry telemetry){
        this.telemetry = telemetry;
        flywheelMotor = new Caching_Motor(map, "shooter");
        flicker = new Flicker(map, telemetry);
        flap = new Caching_Servo(map, "flap");
        rotator = new Caching_Motor(map, "rotator", 1e-6);
        stopper = new Caching_Servo(map, "stopper");
        stopper.setPosition(stopperIn);
        flap.setPosition(0.0);

        rotator.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        write();

        ROTATOR_MIN = Math.toRadians(117.5);
        ROTATOR_MAX = Math.toRadians(170);
        ROTATOR_0 = Math.toRadians(145);

        flywheelPIDController = new PIDFController(new PIDCoefficients(kpF, kiF, kdF));
        feedForward = new SimpleMotorFeedforward(kS, kV);

        rotatorPIDController = new PIDFController(new PIDCoefficients(kpR, kiR, kdR));
        rotatorPIDController2 = new PIDFController(new PIDCoefficients(kpR_2, kiR_2, kdR_2));
        //rotatorPIDController2.setOutputBounds(-0.5, 0.5);
        //rotatorPIDController.setOutputBounds(-0.5, 0.5);
    }

    // Writing to all the motors and servos
    public void write(){
        flicker.write();
        flywheelMotor.write();
        rotator.write();
        flap.write();
        stopper.write();
    }

    public double getFlapPos(double distance){
        double newDist = Range.clip(distance, 75, 105);

        double a = -4.7272727272622e-8 * Math.pow(newDist, 4);
        a += 1.8529292929256e-5 * Math.pow(newDist, 3);
        a += -0.0026813333333283 * Math.pow(newDist, 2);
        a += 0.16957315295787 * newDist;
        a += -3.054402380946;

        /*double[] arr = new double[] {0, 1.14185, -0.223794, -0.0523778,
                0.0147242, -0.000758333};

        double a = 0;

        for(int i = 0; i < arr.length; i++){
            a += arr[i] * Math.pow(newDist, i + 1);
        }*/

        return Range.clip(a, FLAP_MIN, FLAP_MAX);
    }

    public double getFlapPosPowerShot(double distance){
        /*double newDist = Range.clip(distance, 75, 105);
        double a = 9.8181818181719e-8 * Math.pow(newDist, 4);
        a += -3.4723232323197e-5 * Math.pow(newDist, 3);
        a += 0.0045869999999952 * Math.pow(newDist, 2);
        a += -0.26865109668081 * newDist;
        a += 6.7524999999936;*/

        return 0.905;//Range.clip(a + 0.063, FLAP_MIN, FLAP_MAX);
    }

    public double getFlapPosPowerShotTeleOp(double distance){
        double newDist = Range.clip(distance, 75, 105);
        double a = -5.0335522203219e-8 * Math.pow(newDist, 4);
        a += 1.8991015529528e-5 * Math.pow(newDist, 3);
        a += -0.002663956562367633 * Math.pow(newDist, 2);
        a += 0.16451119506176 * newDist;
        a += -2.8980351639951;

        /*double[] arr = new double[] {-3.98641555e-15,  6.16910146e-07,  3.83054854e-05, -8.13602663e-07,
                4.57405101e-09};

        double a = 0.8481613278756606;

        for(int i = 0; i < arr.length; i++){
            a += arr[i] * Math.pow(newDist, i + 1);
        }*/

        return Range.clip(a, FLAP_MIN, FLAP_MAX);
    }

    public void updateData(RevBulkData data2){
        this.data2 = data2;
    }

    public void resetPID(){
        flywheelPIDController.reset();
    }

    public void resetRotatorPID(){
        rotatorPIDController.reset();
        rotatorPIDController2.reset();
    }

    public void setRotator(Pose2d currentPos) {
        //double targetangle = (Robot.isBlue() ? -1 : 1) * Math.atan2(((midGoalToggle ? Robot.ULTIMATE_GOAL2_POS.getX() : Robot.ULTIMATE_GOAL_POS.getX()) - currentPos.getX()), (((midGoalToggle ? Robot.ULTIMATE_GOAL2_POS.getY() : Robot.ULTIMATE_GOAL_POS.getY()) - currentPos.getY())));
        double targetangle = Math.atan2((Robot.ULTIMATE_GOAL_POS.getX() - ((Robot.isBlue() ? -1 : 1) * currentPos.getX())), ((Robot.ULTIMATE_GOAL_POS.getY() - currentPos.getY())));
        double heading = (Robot.isBlue() ? -currentPos.getHeading() : currentPos.getHeading()) + (Robot.isBlue() ? (2 * Math.PI) : 0);

        /*if(heading < 2 * Math.PI && heading >= Math.PI){
            heading -= 2 * Math.PI;
        }*/

        double offset = targetangle - heading;

        setRotator((-offset - rotatorOffset), false);

        telemetry.addData("IS BLUE?", Robot.isBlue());
        telemetry.addData("OFFSET NEW", Math.toDegrees(-offset));
        telemetry.addData("ULTIMATE GOAL POS NEW", Robot.ULTIMATE_GOAL_POS);
        telemetry.addData("Target Position NEW", Math.toDegrees(targetangle));
        telemetry.addData("Current Pos NEW", currentPos);
    }

    public void resetRotator(){
        rotator.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setRotator(int powerShot, Pose2d currentPos, boolean largeTarget) {
        double targetangle;

        switch(powerShot){
            case 1:
                targetangle = Math.atan2((Robot.POWER_SHOT_R.getX() - ((Robot.isBlue() ? -1 : 1) * currentPos.getX())), (Robot.POWER_SHOT_R.getY() - currentPos.getY()));
                break;
            case 2:
                targetangle = Math.atan2((Robot.POWER_SHOT_M.getX() - ((Robot.isBlue() ? -1 : 1) * currentPos.getX())), (Robot.POWER_SHOT_M.getY() - currentPos.getY()));
                break;
            case 3:
                targetangle = Math.atan2((Robot.POWER_SHOT_L.getX() - ((Robot.isBlue() ? -1 : 1) * currentPos.getX())), (Robot.POWER_SHOT_L.getY() - currentPos.getY()));
                break;
            default:
                 targetangle = Math.atan2((Robot.ULTIMATE_GOAL_POS.getX() - ((Robot.isBlue() ? -1 : 1) * currentPos.getX())), (Robot.ULTIMATE_GOAL_POS.getY() - currentPos.getY()));
                 break;
        }

        telemetry.addData("Target Position NEW", Math.toDegrees(targetangle));
        telemetry.addData("Current Pos NEW", currentPos);
        telemetry.addData("IS BLUE?", Robot.isBlue());

        double heading = (Robot.isBlue() ? -currentPos.getHeading() : currentPos.getHeading()) + (Robot.isBlue() ? (2 * Math.PI) : 0);

        /*if(heading <  2 * Math.PI && heading >= Math.PI){
            heading -= 2 * Math.PI;
        }*/

        double offset = targetangle - heading;

        setRotator((-offset - rotatorOffset), largeTarget);
    }

    public void setRotator(int powerShot, Pose2d currentPos){
        setRotator(powerShot, currentPos, true);
    }
    
    private void setRotatorPower(double power){
        rotator.setPower(power);
    }

    public void setFlap(double pos){
        flap.setPosition(pos, 1e-6);
    }

    //Target is 0-(2*PI) and 0-(-2*PI)
    //Negative values represent counter clockwise and positive values represent clockwise
    public void setRotator(double target, boolean largeTarget) {
        if(largeTarget){
            telemetry.addLine("LARGE TARGET");
            if(target > Math.PI){
                target -= 2 * Math.PI;
            }

            target = Range.clip(target + ROTATOR_0, ROTATOR_MIN, ROTATOR_MAX);
            rotatorPIDController2.setTargetPosition(target);

            setRotatorPower(-rotatorPIDController2.update(getRotatorPos()));

            if(Math.abs(rotatorPIDController2.getLastError()) < Math.toRadians(0.25)){
                telemetry.addLine("IN RANGE!");
                rotatorPIDController2.reset();
            }else{
                telemetry.addLine("REACHING TARGET...");
            }
        }else{
            telemetry.addLine("SMALL TARGET");
            if(target > Math.PI){
                target -= 2 * Math.PI;
            }

            target = Range.clip(target + ROTATOR_0, ROTATOR_MIN, ROTATOR_MAX);
            rotatorPIDController.setTargetPosition(target);

            double power = -rotatorPIDController.update(getRotatorPos());

            setRotatorPower(power);

            if(Math.abs(rotatorPIDController.getLastError()) < Math.toRadians(0.25)){
                telemetry.addLine("IN RANGE!");
                rotatorPIDController.reset();
            }else{
                telemetry.addLine("REACHING TARGET...");
            }
        }
    }

    public void setFlywheelVelocity(double targetVelo, double currentVelo){
        flywheelPIDController.setTargetPosition(targetVelo);

        setFlywheelPower(flywheelPIDController.update(currentVelo) + (feedForward.calculate(targetVelo)/2800));
    }

    public void setFlywheelPower(double value){
        flywheelMotor.setPower(-value);
    }

    public double getFlywheelVelcoity(RevBulkData data){
        if(data != null){
            return -data.getMotorVelocity(flywheelMotor.motor);
        }else{
            return 0;
        }
    }

    public double getRotatorPos(){
        //return (-(data2.getMotorCurrentPosition(rotator.motor)/1440.0) * (2 * Math.PI)) + ROTATOR_0;
        return (-(data2.getMotorCurrentPosition(rotator.motor)/1440.0) * (2 * Math.PI)) + Math.toRadians(114);
    }

    public double getRotatorError(){
        return rotatorPIDController.getLastError();
    }
    public double getRotatorError2(){
        return rotatorPIDController2.getLastError();
    }

    public void operate(GamepadEx gamepad1Ex, GamepadEx gamepad2Ex, Pose2d currentPos, TelemetryPacket packet){
        double flywheelVelo = getFlywheelVelcoity(data2);

        telemetry.addData("Flywheel Velocity", flywheelVelo);
        packet.put("Flywheel Velocity", flywheelVelo);

        telemetry.addData("Dist to Ultimate Goal", currentPos.vec().distTo(Robot.ULTIMATE_GOAL_POS));

        if(gamepad1Ex.isPress(GamepadEx.Control.dpad_up)){
            speedToggle = !speedToggle;
        }

        if(gamepad1Ex.isPress(GamepadEx.Control.a)){
            powerShotToggle++;
            powerShotToggle %= 4;
            rotatorPIDController.reset();
            rotatorPIDController2.reset();
            bigPID = true;
        }

        if(gamepad2Ex.isPress(GamepadEx.Control.dpad_up)){
            flapOffset += 0.005;
        }

        if(gamepad2Ex.isPress(GamepadEx.Control.dpad_down)){
            flapOffset -= 0.005;
        }

        if(gamepad2Ex.isPress(GamepadEx.Control.dpad_left)){
            rotatorOffset -= Math.toRadians(0.5);
        }

        if(gamepad2Ex.isPress(GamepadEx.Control.dpad_right)){
            rotatorOffset += Math.toRadians(0.5);
        }

        if(gamepad2Ex.isPress(GamepadEx.Control.b)){
            stopperToggle = !stopperToggle;
            if(stopperToggle){
                stopper.setPosition(stopperOpen);
            }else{
                stopper.setPosition(stopperIn);
            }
        }

        if(gamepad1Ex.isPress(GamepadEx.Control.right_trigger) || gamepad2Ex.isPress(GamepadEx.Control.a)) {
            flywheelToggle = !flywheelToggle;
        }

        if (flywheelToggle) {
            //if(powerShotToggle == 0) {
                setFlywheelVelocity(speedToggle ? 1900 : flywheelTargetVelo, flywheelVelo);
            /*}else{
                setFlywheelVelocity(1350, flywheelVelo);
            }*/

            //Only occurs one to reset the flicker timer and position
            if(gamepad1Ex.isPress(GamepadEx.Control.left_trigger) || gamepad2Ex.isPress(GamepadEx.Control.left_trigger)){
                flicker.reset();
            }

            //If you hold the left trigger then it flicks continuously
            if(gamepad1Ex.gamepad.left_trigger > 0.5 || gamepad2Ex.gamepad.left_trigger > 0.5){
                if(timer.time() > 0.25){
                    if(powerShotToggle != 0){
                        flicker.flickOnce();
                    }else {
                        flicker.flick();
                    }
                }else{
                    flicker.resetTime();
                }

                stopper.setPosition(stopperOpen);
            }else{
                if(!stopperToggle) {
                    stopper.setPosition(stopperIn);
                }
                timer.reset();
                flicker.setPos(Flicker.inPos);
            }
        }else{
            Intake.pause = false;
            flywheelMotor.setPower(0.0);
        }

        if(gamepad1Ex.isPress(GamepadEx.Control.b)){
            resetRotatorPID();
            flapToggle = !flapToggle;
        }

        if(powerShotToggle == 0){
            if(flapToggle){
                setFlap(0.91);
            } else {
                setFlap(getFlapPos(Robot.ULTIMATE_GOAL_POS.distTo(currentPos.vec())) + flapOffset);
            }

            telemetry.addData("Dist to Ultimate Goal", currentPos.vec().distTo(Robot.ULTIMATE_GOAL_POS));
        }else if(powerShotToggle == 1){
            setFlap(getFlapPosPowerShotTeleOp(Robot.POWER_SHOT_R.distTo(currentPos.vec())) + flapOffset);

            telemetry.addData("Dist to Right Power Shot", currentPos.vec().distTo(Robot.POWER_SHOT_R));
        }else if(powerShotToggle == 2){
            setFlap(getFlapPosPowerShotTeleOp(Robot.POWER_SHOT_M.distTo(currentPos.vec())) + flapOffset);

            telemetry.addData("Dist to Middle Power Shot", currentPos.vec().distTo(Robot.POWER_SHOT_M));
        }else if(powerShotToggle == 3){
            setFlap(getFlapPosPowerShotTeleOp(Robot.POWER_SHOT_L.distTo(currentPos.vec())) + flapOffset);

            telemetry.addData("Dist to Left Power Shot", currentPos.vec().distTo(Robot.POWER_SHOT_L));
        }

        telemetry.addData("Flap Offset", flapOffset);

        if(flapToggle){
            setRotator(0, true);
        }else{
            if(bigPID){
                setRotator(powerShotToggle, currentPos);
                if(Math.toDegrees(Math.abs(rotatorPIDController2.getLastError())) < 0.5){
                    bigPID = false;
                }
            }else{
                setRotator(powerShotToggle, currentPos, false);
            }
        }

        //Flap Regression Tuning
        //_________________________________________________________
        /*if(gamepad2Ex.gamepad.dpad_up){
            flapTesterPos += 0.0001;
        }

        if(gamepad2Ex.gamepad.dpad_down){
            flapTesterPos -= 0.0001;
        }

        setFlap(flapTesterPos);*/
        //----------------------------------------------------------

        telemetry.addData("Rotator Error 1", Math.toDegrees(getRotatorError()));
        telemetry.addData("Rotator Error 2", Math.toDegrees(getRotatorError2()));
        telemetry.addData("Rotator Angle", Math.toDegrees(getRotatorPos()));
        telemetry.addData("Flap Position", flapTesterPos);
    }
}