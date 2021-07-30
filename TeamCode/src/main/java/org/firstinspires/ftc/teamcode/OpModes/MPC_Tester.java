package org.firstinspires.ftc.teamcode.OpModes;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.MPC.control.MPCSolver;
import org.firstinspires.ftc.teamcode.MPC.control.Obstacle;
import org.firstinspires.ftc.teamcode.MPC.control.RunnableMPC;
import org.firstinspires.ftc.teamcode.MPC.drivers.Motor;
import org.firstinspires.ftc.teamcode.MPC.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.MPC.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.MPC.physics.InvalidDynamicModelException;
import org.firstinspires.ftc.teamcode.MPC.physics.MecanumDriveModel;
import org.firstinspires.ftc.teamcode.MPC.physics.MotorModel;
import org.firstinspires.ftc.teamcode.MPC.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.MPC.util.TimeUnits;
import org.firstinspires.ftc.teamcode.MPC.util.TimeUtil;
import org.firstinspires.ftc.teamcode.MPC.util.Util;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

//If you want to be able to run the OpMode then you need this. The name is optional.
@Autonomous
@Disabled
//Make sure that you extend to OpMode to access the robot properly.
public class MPC_Tester extends LinearOpMode {
    private static final boolean MOTORS_CONFIGURED = false;

    private Pose2d fieldPosition = new Pose2d(0d, 0d, new Rotation2d(0d, false));

    private ArrayList<SimpleMatrix> desiredStates = new ArrayList<>();

    private SimpleMatrix state;
    private SimpleMatrix input;

    private MecanumDriveModel driveModel;
    private MPCSolver mpcSolver;
    private RunnableMPC runnableMPC;

    private TimeProfiler timeProfiler;
    private TimeProfiler elapsedTime;

    private com.acmerobotics.roadrunner.geometry.Pose2d prevPose = new com.acmerobotics.roadrunner.geometry.Pose2d(0, 0, 0);
    int robotState = 0;

    List<Obstacle> obstacles = new ArrayList<>();

    //FileWriter myWriter;
    private boolean timerStarted = false;
    double lastResetTime = 0d;
    Robot robot;

    private final double dt = 0.002d;

    static {
        TimeUtil.isUsingComputer = false;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);

        telemetry.addLine("Initializing... Do not start!");
        telemetry.update();

        robot.localizer.reset();

        /*try {
            myWriter = new FileWriter("positions.txt");
        } catch (IOException e) {
            e.printStackTrace();
        }*/
        state = new SimpleMatrix(6, 1);
        input = new SimpleMatrix(4, 1);

        timeProfiler = new TimeProfiler(false);
        elapsedTime = new TimeProfiler(false);

        driveModel = new MecanumDriveModel(0.01,
                17.872,
                0.207,
                0.000238,
                2.49e-5,
                0.6226,
                0.048,
                0.2286, 0.2286, 0.2286, 0.2286,
                MotorModel.generateMotorModel(Motor.NEVEREST_20, 4, 3.0/4.0, null));

        //obstacles.add(new Obstacle(22.5,22.5, 3d, 200d));

        /*mpcSolver = new MPCSolver(1000, dt, SimpleMatrix.diag(100d, 10, 100d, 10, 100d, 10),
                SimpleMatrix.diag(1000d, 50d, 200d, 25d, 10d, 1d), SimpleMatrix.diag(1d, 1d, 1d, 1d), driveModel, obstacles);*/
        mpcSolver = new MPCSolver(1000, dt, SimpleMatrix.diag(50d, 5, 50d, 5, 50d, 5),
                SimpleMatrix.diag(100d, 10d, 100d, 10d, 5d, 0d), SimpleMatrix.diag(3d, 3d, 3d, 3d), driveModel, obstacles);

        desiredStates.add(Util.convertPoseToState(new Pose2d(48d, 48d, new Rotation2d(Math.toRadians(90d), false))));
        //desiredStates.add(Util.convertPoseToState(new Pose2d(6d + 52, -(10d - 50), new Rotation2d(Math.toRadians(0d), false))));
        //desiredStates.add(Util.convertPoseToState(new Pose2d(100d, 0d, new Rotation2d(Math.toRadians(0d), false))));
        //desiredStates.add(Util.convertPoseToState(new Pose2d(-27d + 52, -(55d - 50), new Rotation2d(Math.toRadians(180d), false))));
        //desiredStates.add(Util.convertPoseToState(new Pose2d(48d + 52,-(61d - 50), new Rotation2d(Math.toRadians(0d), false))));
        //desiredStates.add(Util.convertPoseToState(new Pose2d(18d + 52, -(43d - 50), new Rotation2d(Math.toRadians(0d), false))));

        try {
            mpcSolver.initializeAndIterate(5, state, desiredStates.get(0));
        } catch (InvalidDynamicModelException e) {
            e.printStackTrace();
        }

        runnableMPC = new RunnableMPC(5, mpcSolver, () -> state, desiredStates.get(0), obstacles);
        new Thread(runnableMPC).start();

        telemetry.addLine("Ok, done initializing. You can start the OpMode now.");
        telemetry.update();

        waitForStart();

        timeProfiler.start();
        elapsedTime.start();

        runnableMPC.resetTimer();

        while(opModeIsActive() && !isStopRequested()){
            robot.updateBulkData();
            robot.updatePos();

            double dt = timeProfiler.getDeltaTime(TimeUnits.SECONDS, true);

            if(desiredStates.size() > 1 && Util.convertStateToPose(desiredStates.get(0)).distance(getFieldPosition()) < 2d
                    && Math.abs(desiredStates.get(0).get(4) - getFieldPosition().getRotation().getRadians()) < Math.toRadians(1d)) {
                if(!timerStarted) {
                    timerStarted = true;
                    elapsedTime.reset();
                }

                if(elapsedTime.getDeltaTime(TimeUnits.SECONDS, false) > 0d || desiredStates.size() < 3) {
                    desiredStates.remove(0);
                    runnableMPC.setDesiredState(desiredStates.get(0));
                    timerStarted = false;
                }
            }

            if(dt != 0){

                MPCSolver updatedController = runnableMPC.getUpdatedMPC();
                if(updatedController != null) {
                    mpcSolver = updatedController;
                }

                try {
                    input = mpcSolver.getOptimalInput((int)(runnableMPC.controllerElapsedTime()/dt), state);
                } catch(Exception e) {
                    e.printStackTrace();
                }

                //angle wrap heading
                double heading = robot.localizer.getAbsoluteHeading();

                com.acmerobotics.roadrunner.geometry.Pose2d velocities = new com.acmerobotics.roadrunner.geometry.Pose2d((robot.getPos().getX() - prevPose.getX())/dt, (robot.getPos().getY() - prevPose.getY())/dt, (heading - prevPose.getHeading())/dt);

                //angle wrap angular velocity
                double omega = velocities.getHeading();

                state = new SimpleMatrix(6, 1, true, new double[]{robot.getPos().getY()/39.37, velocities.getY()/39.37, -robot.getPos().getX()/39.37, -velocities.getX()/39.37, heading, -omega});
                //state = driveModel.simulate(state, input, dt);

                applyInput();

                telemetry.addData("1:", "Time taken for " + runnableMPC.getIterations() + " iterations for MPC controller (ms): " + runnableMPC.getPolicyLag());
                telemetry.addData("2:", state);
                telemetry.addData("Motor actuations: ", input);

                prevPose = new com.acmerobotics.roadrunner.geometry.Pose2d(robot.getPos().getX(), robot.getPos().getY(), heading);
            }

            telemetry.addData("Error Position", robot.getPos().vec().distTo(new Vector2d(-48, 48)));
            telemetry.addData("Error Heading", Math.abs(robot.getPos().getHeading()));

            telemetry.update();
        }
    }

    public double scaleInput(double input, double kS, double maxPower){
        double factor = maxPower - kS;
        return factor * input + Math.signum(input) * kS;
    }

    /*@Override
    public void stop(){
        try {
            myWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }*/

    public Pose2d getFieldPosition() {
        return new Pose2d(state.get(0) / 0.0254d, state.get(2) / 0.0254d, new Rotation2d(state.get(4), false));
    }

    private void applyInput() {
        robot.drive.setPower(
                /*scaleInput(input.get(0), 0.08, 1.0),
                scaleInput(input.get(2), 0.08, 1.0),
                scaleInput(input.get(1), 0.08, 1.0),
                scaleInput(input.get(3), 0.08, 1.0));*/
                input.get(0),
                input.get(2),
                input.get(1),
                input.get(3));
        robot.drive.write();
    }
}
