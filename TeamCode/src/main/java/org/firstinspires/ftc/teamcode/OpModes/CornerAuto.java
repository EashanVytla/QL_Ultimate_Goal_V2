package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Flicker;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePusuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePusuit.RobotMovement;

import java.util.ArrayList;

import androidx.core.view.TintableBackgroundView;

@Autonomous
public class CornerAuto extends LinearOpMode {
    int state = 0;
    ElapsedTime elapsedTime;
    boolean gtp = false;
    Robot robot;
    private final Pose2d ZONE_1_POS = new Pose2d(11, 72, Math.toRadians(0));
    private final Pose2d ZONE_2_POS = new Pose2d(-8.5, 95.5, Math.toRadians(0));
    private final Pose2d ZONE_4_POS = new Pose2d(11, 119.5, Math.toRadians(325));

    private final Pose2d CLEAR_STACK = new Pose2d(21, 25);
    private final Pose2d SHOOT_HIGH_GOAL = new Pose2d(23, 51);
    private final Pose2d PREPARE_STACK = new Pose2d(24, 26, Math.toRadians(270));
    private final Pose2d PREPARE_STACK2 = new Pose2d(-18, 28, Math.toRadians(270));
    private final Pose2d TURN_TO_STACK = new Pose2d(-18, 18, 0);
    private final Pose2d STRAFE_INTAKE_1 = new Pose2d(-10, 27, 0);
    private final Pose2d STRAFE_INTAKE_2 = new Pose2d(6, 27, 0);

    private final Pose2d PARK = new Pose2d(2, 67, Math.toRadians(0));

    private int ringCase = 0;

    private boolean readyForShots = false;

    @Override
    public void runOpMode() {
        elapsedTime = new ElapsedTime();
        robot = new Robot(hardwareMap, telemetry);
        robot.localizer.reset();

        robot.intake.initAuto();

        robot.intake.write();

        robot.shooter.resetRotator();

        if(Robot.isBlue()) {
            Robot.ULTIMATE_GOAL_POS = new Vector2d(3, 136);
        }

        robot.setStartPose(new Pose2d(Robot.isBlue() ? -19 : 19, 0));

        robot.initializeWebcam();

        while(!isStarted() && !isStopRequested()){;
            ringCase = robot.getRingStackCase();
            telemetry.addData("Ring Case", ringCase);

            if(gamepad1.a){
                robot.blue();
                robot.wobbleGoal.init();
            }

            if(gamepad1.y){
                robot.red();
                robot.wobbleGoal.init();
            }

            if(Robot.isBlue()){
                telemetry.addData("BLUE SIDE AUTO...", "TEEHEE");
            }else{
                telemetry.addData("RED SIDE AUTO...", "TEEHEE");
            }

            telemetry.update();
            elapsedTime.reset();
        }

        robot.stopWebcam();

        waitForStart();

        elapsedTime.startTime();

        robot.shooter.resetRotator();

        while(opModeIsActive()){
            robot.updateBulkData();
            ArrayList<CurvePoint> points = new ArrayList<>();

            double flywheelVelo = robot.shooter.getFlywheelVelcoity(robot.getData2());

            telemetry.addData("Flywheel Velocity", flywheelVelo);

            switch(state){
                case 0:
                    if(elapsedTime.time() >= 0.1){
                        robot.shooter.stopper.setPosition(robot.shooter.stopperOpen);
                        robot.wobbleGoal.lift();
                        robot.wobbleGoal.clamp();
                        elapsedTime.reset();
                        RobotMovement.resetIndex();
                        state++;
                    }

                    robot.intake.release();

                    break;
                case 1:
                    robot.shooter.setRotator(robot.getPos());
                    points.add(new CurvePoint(CLEAR_STACK, 1d, 1d, 25));
                    points.add(new CurvePoint(SHOOT_HIGH_GOAL, 0.75d, 1d, 25));
                    robot.shooter.setFlap(robot.shooter.getFlapPos(robot.getPos().vec().distTo(Robot.ULTIMATE_GOAL_POS)));
                    robot.shooter.setFlywheelVelocity(2000, robot.shooter.getFlywheelVelcoity(robot.getData2()));

                    if(readyForShots){
                        if(elapsedTime.time() > 1.5){
                            robot.shooter.flicker.setPos(Flicker.inPos);
                            robot.shooter.setFlywheelPower(0.0);
                            readyForShots = false;
                            state++;
                        }

                        /*if(Math.abs(robot.shooter.getRotatorError()) < Math.toRadians(0.25)){
                            robot.shooter.resetRotatorPID();
                        }*/

                        if(elapsedTime.time() > 0.25){
                            robot.shooter.flicker.flick();
                        }else{
                            robot.shooter.flicker.resetTime();
                        }

                        robot.shooter.stopper.setPosition(robot.shooter.stopperOpen);
                    }else{
                        if(robot.getPos().vec().distTo(SHOOT_HIGH_GOAL.vec()) < 1.0 && robot.shooter.getFlywheelVelcoity(robot.getData2()) >= 1980 && Math.abs(robot.shooter.getRotatorError()) < Math.toRadians(0.75)){
                            robot.shooter.resetRotatorPID();
                            readyForShots = true;
                        }

                        robot.shooter.setRotator(0, robot.getPos());

                        elapsedTime.reset();
                    }

                    break;
                case 2:
                    robot.shooter.flicker.setPos(Flicker.inPos);
                    gtp = true;
                    switch (ringCase){
                        case 0:
                            points.add(new CurvePoint(ZONE_1_POS, 1d, 1d, 25));
                            break;
                        case 1:
                            points.add(new CurvePoint(ZONE_2_POS, 1d, 1d, 25));
                            break;
                        case 4:
                            points.add(new CurvePoint(ZONE_4_POS, 1d, 1d, 25));
                            break;
                    }
                    if(points.get(points.size() - 1).toVec().distTo(robot.getPos().vec()) < 2.0){
                        elapsedTime.reset();
                        state++;
                    }
                    break;
                case 3:
                    if(elapsedTime.time() > 1.5){
                        if(ringCase == 0){
                            state = 12;
                        }else if(ringCase == 1){
                            state = 13;
                        }else if(ringCase == 4){
                            state++;
                        }
                    }else if(elapsedTime.time() > 1.0){
                        robot.wobbleGoal.lift();
                    }else if(elapsedTime.time() > 0.5){
                        robot.wobbleGoal.release();
                    }else{
                        robot.wobbleGoal.down();
                    }
                    break;
                case 4:
                    gtp = true;
                    if(PREPARE_STACK.vec().distTo(robot.getPos().vec()) < 20){
                        points.add(new CurvePoint(PREPARE_STACK, 1d, 1d, 25));
                    }else{
                        points.add(new CurvePoint(new Pose2d(PREPARE_STACK.getX(), PREPARE_STACK.getY()), 1d, 1d, 25));
                    }
                    robot.wobbleGoal.clamp();
                    if(Math.abs(robot.getPos().getHeading() - PREPARE_STACK.getHeading()) < Math.toRadians(5) && robot.getPos().vec().distTo(PREPARE_STACK.vec()) < 2.5){
                        if(Robot.isBlue()){
                            robot.wobbleGoal.servo_liftLeft.setPosition(0.002);
                        }else{
                            robot.wobbleGoal.servo_liftRight.setPosition(0.002);
                        }
                        elapsedTime.reset();
                        state++;
                    }
                    break;
                case 5:
                    gtp = true;
                    if(elapsedTime.time() < 3.0 || elapsedTime.time() > 3.5){
                        points.add(new CurvePoint(PREPARE_STACK2, 0.2d, 1d, 25));
                    }
                    if(robot.getPos().vec().distTo(PREPARE_STACK2.vec()) < 1.0){
                        state++;
                    }
                    break;
                case 6:
                    gtp = true;
                    points.add(new CurvePoint(TURN_TO_STACK, 1d, 1d, 25));
                    if(Math.abs(robot.getPos().getHeading() - TURN_TO_STACK.getHeading()) < Math.toRadians(2) && robot.getPos().vec().distTo(TURN_TO_STACK.vec()) < 1.0){
                        state++;
                    }
                    break;
                case 7:
                    gtp = true;
                    points.add(new CurvePoint(new Pose2d(TURN_TO_STACK.getX(), STRAFE_INTAKE_1.getY()), 1d, 1d, 5));

                    robot.intake.setPower(1.0);
                    if(robot.getPos().vec().distTo(new Pose2d(TURN_TO_STACK.getX(), STRAFE_INTAKE_1.getY()).vec()) < 2 && Math.abs(robot.getPos().getHeading() - STRAFE_INTAKE_1.getHeading()) <= Math.toRadians(3)){
                        state++;
                        elapsedTime.reset();
                    }
                    break;
                case 8:
                    robot.shooter.flicker.setPos(Flicker.inPos);
                    robot.shooter.setFlap(robot.shooter.getFlapPos(robot.getPos().vec().distTo(Robot.ULTIMATE_GOAL_POS)));
                    points.add(new CurvePoint(STRAFE_INTAKE_1, 0.5d, 1d, 5));
                    if(robot.getPos().vec().distTo(STRAFE_INTAKE_1.vec()) < 2){
                        if(elapsedTime.time() > 1.5){
                            robot.shooter.resetRotatorPID();
                            state++;
                            elapsedTime.reset();
                        }
                    }else{
                        elapsedTime.reset();
                    }
                    break;
                case 9:
                case 11:
                    robot.shooter.setFlap(robot.shooter.getFlapPos(robot.getPos().vec().distTo(Robot.ULTIMATE_GOAL_POS)));
                    robot.intake.setPower(0.0);
                    robot.shooter.setFlywheelVelocity(2000, robot.shooter.getFlywheelVelcoity(robot.getData2()));

                    if(readyForShots){
                        if(elapsedTime.time() > 1.5){
                            robot.shooter.flicker.setPos(Flicker.inPos);
                            robot.shooter.setFlywheelPower(0.0);
                            readyForShots = false;
                            state++;
                        }

                        /*if(Math.abs(robot.shooter.getRotatorError()) < Math.toRadians(0.25)){
                            robot.shooter.resetRotatorPID();
                        }*/

                        if(elapsedTime.time() > 0.25){
                            robot.shooter.flicker.flick();
                        }else{
                            robot.shooter.flicker.resetTime();
                        }

                        robot.shooter.stopper.setPosition(robot.shooter.stopperOpen);
                    }else{
                        if(robot.shooter.getFlywheelVelcoity(robot.getData2()) >= 1980 && Math.abs(robot.shooter.getRotatorError()) < Math.toRadians(0.75)){
                            robot.shooter.resetRotatorPID();
                            readyForShots = true;
                        }

                        robot.shooter.setRotator(0, robot.getPos());

                        elapsedTime.reset();
                    }
                    break;
                case 10:
                    robot.shooter.flicker.setPos(Flicker.inPos);
                    robot.shooter.setFlap(robot.shooter.getFlapPos(robot.getPos().vec().distTo(Robot.ULTIMATE_GOAL_POS)));
                    gtp = true;
                    points.add(new CurvePoint(STRAFE_INTAKE_2, 0.5d, 1d, 25));
                    robot.intake.setPower(1.0);
                    if(robot.getPos().vec().distTo(STRAFE_INTAKE_2.vec()) < 2){
                        if(elapsedTime.time() > 1.5){
                            robot.shooter.resetRotatorPID();
                            state++;
                            elapsedTime.reset();
                        }
                    }else{
                        elapsedTime.reset();
                    }
                    break;
                case 12:
                    robot.shooter.flicker.setPos(Flicker.inPos);
                    robot.shooter.setFlywheelPower(0.0);
                    gtp = true;
                    points.add(new CurvePoint(PARK, 0.5d, 0.5d, 25));

                    if(PARK.vec().distTo(robot.getPos().vec()) < 2.5){
                        robot.wobbleGoal.down();
                        robot.wobbleGoal.clamp();
                    }

                    break;
                case 13:
                    gtp = false;
                    switch (ringCase){
                        case 0:
                            points.add(new CurvePoint(ZONE_1_POS, 1d, 1d, 10));
                            break;
                        case 1:
                            points.add(new CurvePoint(ZONE_2_POS, 1d, 1d, 5));
                            points.add(new CurvePoint(new Pose2d(ZONE_2_POS.getX() + 1, 75), 1d, 1d, 10));
                            points.add(new CurvePoint(new Pose2d(PREPARE_STACK.getX() + 1, 75), 1d, 1d, 10));
                            break;
                        case 4:
                            points.add(new CurvePoint(ZONE_4_POS, 1d, 1d, 10));
                            break;
                    }

                    points.add(new CurvePoint(new Pose2d(PREPARE_STACK.getX(), 14.772 + 1), 1d, 1d, 25));
                    points.add(new CurvePoint(new Pose2d(1.611, 14.772), 1d, 1d,   25));
                    if(new Pose2d(1.611, 14.772).vec().distTo(robot.getPos().vec()) <= 1.0){
                        state++;
                    }
                    break;
                case 14:
                    robot.intake.setPower(1.0);
                    gtp = true;
                    points.add(new CurvePoint(new Pose2d(1.611, 49.0), 0.2d, 1d, 25));
                    if(new Pose2d(1.611, 49.0).vec().distTo(robot.getPos().vec()) < 2.0){
                        robot.shooter.resetRotatorPID();
                        state = 11;
                    }
                    break;
            }

            if(points.size() != 0){
                if(!gtp){
                    RobotMovement.followCurve(points, robot, telemetry);
                }else{
                    robot.GoTo(points.get(points.size() - 1).toPose(),new Pose2d(points.get(points.size() - 1).moveSpeed, points.get(points.size() - 1).moveSpeed, points.get(points.size() - 1).turnSpeed));
                }
            }else{
                robot.drive.setPower(0, 0, 0);
                robot.drive.write();
                robot.updatePos();
            }

            robot.wobbleGoal.write();
            robot.shooter.write();
            robot.intake.write();

            telemetry.addData("Rotator Position", robot.shooter.getRotatorPos());

            telemetry.addData("Size", points.size());
            telemetry.addData("GTP", gtp);

            telemetry.addData("Position", robot.getPos());
            telemetry.addData("State", state);
            telemetry.update();
        }
    }
}
