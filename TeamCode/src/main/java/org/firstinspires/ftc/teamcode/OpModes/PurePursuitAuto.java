package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Flicker;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.MPC.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.MPC.util.TimeUnits;
import org.firstinspires.ftc.teamcode.PurePusuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePusuit.RobotMovement;

import java.util.ArrayList;

@Autonomous
public class PurePursuitAuto extends LinearOpMode {
    int state = 0;
    ElapsedTime elapsedTime;
    boolean gtp = false;
    Robot robot;

    @Override
    public void runOpMode() {
        Robot.setContinuous(false);
        elapsedTime = new ElapsedTime();
        robot = new Robot(hardwareMap, telemetry);
        robot.localizer.reset();

        robot.wobbleGoal.servo_grab.setPosition(robot.wobbleGoal.clamp_pos);
        robot.wobbleGoal.servo_lift.setPosition(0.3475);
        robot.wobbleGoal.servo_lift.write();
        robot.wobbleGoal.servo_grab.write();

        robot.shooter.converter.setPosition(0.35);

        robot.intake.barUp();
        robot.intake.write();

        waitForStart();

        elapsedTime.startTime();

        while(opModeIsActive()){
            robot.updateBulkData();
            ArrayList<CurvePoint> points = new ArrayList<>();

            double flywheelVelo = robot.shooter.getFlywheelVelcoity(robot.getData2());

            telemetry.addData("Flywheel Velocity", flywheelVelo);

            switch(state){
                case 0:
                    if(elapsedTime.time() >= 0.1){
                        robot.localizer.reset();
                        robot.wobbleGoal.autoLift();
                        robot.intake.barDown();
                        elapsedTime.reset();
                        RobotMovement.resetIndex();
                        state++;
                    }

                    break;
                case 1:
                    points.add(new CurvePoint(new Pose2d(0, 0, Math.toRadians(0)), 1d, 1d, 25));
                    points.add(new CurvePoint(new Pose2d(-16d, 27d, Math.toRadians(0)), 1d, 1d, 25));
                    points.add(new CurvePoint(new Pose2d(-18.5d, 56.5d, Math.toRadians(0)), 0.5d, 1.0d, 25));

                    robot.shooter.setFlap(0.035 + Shooter.FLAP_MIN);

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.0 && Math.abs(Robot.wrapHeading(robot.getPos().getHeading())) < Math.toRadians(0.5)){
                        if(elapsedTime.time() > 3.0){
                            robot.shooter.stopFlywheel();
                            elapsedTime.reset();
                            robot.shooter.resetPID();
                            robot.drive.resetPID();
                            state++;
                        }else{
                            if(elapsedTime.time() > 2.0){
                                robot.shooter.setRotator(0.4165 + 0.0255);
                                if(elapsedTime.time() > 2.5){
                                    robot.shooter.flicker.setPos(Flicker.inPos);
                                }else if(elapsedTime.time() > 2.25){
                                    robot.shooter.flicker.setPos(Flicker.outPos);
                                }
                            }else if(elapsedTime.time() > 1.0){
                                robot.shooter.setRotator(0.452 + 0.0255);
                                if(elapsedTime.time() > 1.5){
                                    robot.shooter.flicker.setPos(Flicker.inPos);
                                }else if(elapsedTime.time() > 1.25){
                                    robot.shooter.flicker.setPos(Flicker.outPos);
                                }
                            }else{
                                robot.shooter.setRotator(0.48749 + 0.0355);
                                if(elapsedTime.time() > 0.5){
                                    robot.shooter.flicker.setPos(Flicker.inPos);
                                }else if(elapsedTime.time() > 0.25){
                                    robot.shooter.flicker.setPos(Flicker.outPos);
                                }
                            }
                        }
                    }else{
                        elapsedTime.reset();
                    }

                    robot.shooter.setFlywheelVelocity(1750, flywheelVelo);

                    break;
                case 2:
                    points.add(new CurvePoint(new Pose2d(-21d, 56d, Math.toRadians(0)), 1d, 1d, 25));
                    points.add(new CurvePoint(new Pose2d(19, 109.5, Math.toRadians(325)), 1d, 1d, 25));

                    robot.shooter.flicker.setIdlePos(robot.shooter.getRotatorPos());

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.0){
                        if(elapsedTime.time() > 1.0){
                            elapsedTime.reset();
                            robot.drive.resetPID();
                            state++;
                        }else{
                            if(elapsedTime.time() > 0.5){
                                robot.wobbleGoal.release();
                            }
                            robot.wobbleGoal.down();
                        }
                    }else{
                        elapsedTime.reset();
                    }

                    break;
                case 3:
                    points.add(new CurvePoint(new Pose2d(19, 109.5, Math.toRadians(325)), 1d, 1d, 25));
                    points.add(new CurvePoint(new Pose2d(-15, 54, Math.toRadians(0)), 1d, 1d, 25));
                    points.add(new CurvePoint(new Pose2d(-16.5, 18.7, Math.toRadians(0)), 1d, 1d, 25));
                    points.add(new CurvePoint(new Pose2d(1.69, 18.2, Math.toRadians(0)), 1d, 1d, 25));

                    robot.shooter.converter.setPosition(Shooter.continuousModePos);

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.0){
                        if(elapsedTime.time() > 1.0){
                            elapsedTime.reset();
                            robot.drive.resetPID();
                            robot.wobbleGoal.autoLift();
                            state++;
                        }else{
                            if(elapsedTime.time() > 0.5){
                                robot.wobbleGoal.autoLift();
                            }
                            robot.wobbleGoal.down();
                        }
                        robot.wobbleGoal.clamp();
                    }else{
                        robot.wobbleGoal.release();
                        elapsedTime.reset();
                    }

                    break;
                case 4:
                    robot.shooter.setRotator(robot.getPos());

                    points.add(new CurvePoint(new Pose2d(1.69, 18.2, Math.toRadians(0)), 1d, 1d, 25));
                    points.add(new CurvePoint(new Pose2d(3.8, 20.5, Math.toRadians(0)), 1.0, 1d, 25));
                    if(robot.getPos().getY() > 56){
                        robot.intake.setPower(0.0);
                        robot.shooter.stopFlywheel();
                        points.add(new CurvePoint(new Pose2d(12.3, 109.75, Math.toRadians(325)), 1d, 1d, 25));
                    }else if(robot.getPos().getY() > 20){
                        robot.shooter.converter.setPosition(robot.shooter.continuousModePos);
                        robot.shooter.flicker.setIdlePos(robot.shooter.getRotatorPos());

                        robot.shooter.flap.setPosition(robot.shooter.getFlapPos(Robot.ULTIMATE_GOAL_POS.distTo(robot.getPos().vec())));
                        robot.shooter.setFlywheelVelocity(2000, flywheelVelo);
                        robot.intake.setPower(1.0);

                        points.add(new CurvePoint(new Pose2d(12.3, 109.75, Math.toRadians(0)), 0.2d, 1d, 25));
                    }else{
                        robot.shooter.converter.setPosition(robot.shooter.continuousModePos);
                        robot.shooter.flicker.setIdlePos(robot.shooter.getRotatorPos());

                        robot.shooter.flap.setPosition(robot.shooter.getFlapPos(Robot.ULTIMATE_GOAL_POS.distTo(robot.getPos().vec())));
                        robot.shooter.setFlywheelVelocity(2000, flywheelVelo);
                        robot.intake.setPower(1.0);

                        points.add(new CurvePoint(new Pose2d(12.3, 109.75, Math.toRadians(0)), 1d, 1d, 25));
                    }

                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 1.0){
                        if(elapsedTime.time() > 1.0){
                            elapsedTime.reset();
                            robot.drive.resetPID();
                            state++;
                        }else{
                            if(elapsedTime.time() > 0.5){
                                robot.wobbleGoal.release();
                            }
                            robot.wobbleGoal.down();
                        }
                    }else{
                        elapsedTime.reset();
                    }

                    break;
                case 5:
                    robot.intake.barUp();
                    gtp = true;
                    points.add(new CurvePoint(new Pose2d(12.3, 109.75, Math.toRadians(325)), 1d, 1d, 25));
                    points.add(new CurvePoint(new Pose2d(-6.6, 75.3, Math.toRadians(0)), 1d, 1d, 25));

                    robot.wobbleGoal.lift();
                    robot.wobbleGoal.clamp();

                    break;
            }

            if(points.size() != 0){
                if(!gtp){
                    RobotMovement.followCurve(points, robot, telemetry);
                }else{
                    robot.GoTo(points.get(points.size() - 1).toPose(),new Pose2d(1, 1, 1));
                }
            }else{
                robot.drive.setPower(0, 0, 0);
                robot.updatePos();
            }

            robot.wobbleGoal.write();
            robot.shooter.write();
            robot.intake.write();

            telemetry.addData("Size", points.size());

            telemetry.addData("Position", robot.getPos());
            telemetry.addData("State", state);
            telemetry.update();
        }
    }
}
