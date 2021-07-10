package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Flicker;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp(name = "Blue TeleOp")
public class BlueTeleOp extends LinearOpMode {
    Robot robot;
    Shooter shooter;
    GamepadEx gamepad1ex;
    GamepadEx gamepad2ex;

    double kStrafe = 1.0;
    double kVert = 1.0;

    public boolean powerShots = false;
    private ElapsedTime time = new ElapsedTime();
    private boolean powerShotsKickout = false;
    private int powerShotState = 0;

    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);

        robot.blue();

        robot.localizer.k_strafe = kStrafe;
        robot.localizer.k_vert = kVert;

        gamepad1ex = new GamepadEx(gamepad1);
        gamepad2ex = new GamepadEx(gamepad2);
        robot.intake.barDown();

        waitForStart();

        robot.start();
        time.startTime();

        while(opModeIsActive()){
            robot.updateBulkData();

            if(gamepad1ex.isPress(GamepadEx.Control.back)){
                robot.localizer.reset();
            }

            if(gamepad1ex.isPress(GamepadEx.Control.y)){
                time.reset();
                powerShots = !powerShots;
                powerShotsKickout = false;
            }

            if(!powerShots){
                //Regular Drive State
                robot.operate(gamepad1ex, gamepad2ex);
            }else{
                //Power Shots State

                if(gamepad1ex.gamepad.left_stick_y > 0.3 ||
                        gamepad1ex.gamepad.left_stick_x > 0.3 ||
                        gamepad1ex.gamepad.right_stick_y > 0.3 ||
                        gamepad1ex.gamepad.right_stick_x > 0.3){
                    powerShots = false;
                }

                Pose2d target = new Pose2d(-18.5d, 56.5d, Math.toRadians(0));

                robot.GoTo(target, new Pose2d(1.0, 1.0, 1.0));
                robot.shooter.setFlap(0.036 + Shooter.FLAP_MIN);

                if(powerShotsKickout){
                    robot.shooter.setFlywheelVelocity(1750, robot.shooter.getFlywheelVelcoity(robot.getData2()));

                    switch(powerShotState){
                        case 0:
                            if(time.time() > 1.0){
                                powerShotState = 1;
                            }

                            //FIRST POWERSHOT
                            robot.shooter.setRotator(0.48749 + 0.0255);
                            if(time.time() > 0.5){
                                robot.shooter.flicker.setPos(Flicker.inPos);
                            }else if(time.time() > 0.25){
                                robot.shooter.flicker.setPos(Flicker.outPos);
                            }

                            break;
                        case 1:
                            if(time.time() > 2.0){
                                powerShotState = 2;
                            }

                            //SECOND POWERSHOT
                            robot.shooter.setRotator(0.452 + 0.0205);
                            if(time.time() > 1.5){
                                robot.shooter.flicker.setPos(Flicker.inPos);
                            }else if(time.time() > 1.25){
                                robot.shooter.flicker.setPos(Flicker.outPos);
                            }

                            break;
                        case 2:
                            //THIRD POWERSHOT
                            robot.shooter.setRotator(0.4165 + 0.0205);
                            if(time.time() > 2.5){
                                robot.shooter.flicker.setPos(Flicker.inPos);
                            }else if(time.time() > 2.25){
                                robot.shooter.flicker.setPos(Flicker.outPos);
                            }

                            if(time.time() > 3){
                                powerShotState = 0;
                                powerShotsKickout = false;
                                powerShots = false;
                                robot.shooter.flywheelMotor.setPower(0.0);
                                time.reset();
                            }

                            break;
                    }
                }else{
                    if(robot.shooter.getFlywheelVelcoity(robot.getData2()) > 1650 && robot.getPos().vec().distTo(target.vec()) < 1.0 && Math.abs(Robot.wrapHeading(robot.getPos().getHeading())) < Math.toRadians(1)){
                        powerShotsKickout = true;
                        powerShotState = 0;
                    }else{
                        robot.shooter.setFlywheelVelocity(1750, robot.shooter.getFlywheelVelcoity(robot.getData2()));
                        time.reset();
                    }
                }
                robot.shooter.write();
            }

            telemetry.update();
            gamepad1ex.loop();
            gamepad2ex.loop();
        }
    }
}
