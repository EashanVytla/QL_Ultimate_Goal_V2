package org.firstinspires.ftc.teamcode.OpModes;

import android.os.Environment;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.MPC.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.MPC.util.TimeUnits;
import org.firstinspires.ftc.teamcode.MPC.util.TimeUtil;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

public class ExplicitMPC extends LinearOpMode {

    static {
        TimeUtil.isUsingComputer = false;
    }

    private ArrayList<Pose2d> desiredStates = new ArrayList<>();
    int prevIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        ArrayList<Pose2d> positions = new ArrayList<>();
        ArrayList<Integer> timesteps = new ArrayList<>();
        ArrayList<SimpleMatrix> motorActuations = new ArrayList<>();
        ArrayList<Integer> counters = new ArrayList<>();
        Robot robot = new Robot(hardwareMap, telemetry);
        robot.localizer.reset();

        TimeProfiler profiler;

        profiler = new TimeProfiler(false);

        desiredStates.add(new Pose2d(-21, 56d, Math.toRadians(0)));
        desiredStates.add(new Pose2d(14, 109d, Math.toRadians(360 - 45d)));
        desiredStates.add(new Pose2d(1, 15, Math.toRadians(0d)));
        desiredStates.add(new Pose2d(14, 109, Math.toRadians(360 - 45d)));
        desiredStates.add(new Pose2d(-10, 69, Math.toRadians(0d)));

        BufferedReader bufReader = null;
        try {
            bufReader = new BufferedReader(new FileReader(Environment.getExternalStorageDirectory() + "/FIRST/positions.txt"));

            String line = "";

            while (true) {
                line = bufReader.readLine();
                if(line != null){
                    String[] values = line.split(":");
                    int timestep = (int)Float.parseFloat(values[0]);
                    String[] poseString = values[1].split(",");
                    String[] ffString = values[2].split(",");

                    SimpleMatrix ff = new SimpleMatrix(4, 1, true, new double[]{
                            Double.parseDouble(ffString[0]),
                            Double.parseDouble(ffString[1]),
                            Double.parseDouble(ffString[2]),
                            Double.parseDouble(ffString[3])
                    });

                    Pose2d pose = new Pose2d(Double.parseDouble(poseString[0]), Double.parseDouble(poseString[1]), Double.parseDouble(poseString[2]));

                    counters.add(Integer.parseInt(values[3]));
                    positions.add(pose);
                    timesteps.add(timestep);
                    motorActuations.add(ff);
                }else{
                    break;
                }
            }

            bufReader.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }

        Log.i("Size of array: ", String.valueOf(timesteps.size()));

        telemetry.addData("Ready to start", "Ready to start");
        telemetry.update();
        int state = 0;
        boolean paused = false;

        waitForStart();

        profiler.start();
        int timestepIndex = 0;

        while(opModeIsActive() && !isStopRequested()){
            robot.updateBulkData();
            robot.updatePos();

            //double dt = profiler.getDeltaTime(TimeUnits.MILLISECONDS, false);

            //timestepIndex = timeStepSearch(dt, timesteps);
            timestepIndex = posSearch(new Pose2d(robot.getPos().getY(), -robot.getPos().getX(), -robot.getPos().getHeading()), positions);

            if(timestepIndex >= timesteps.size()){
                timestepIndex = timesteps.size() - 1;
            }

            if(timestepIndex < 0){
                timestepIndex = 0;
            }

            if(desiredStates.size() != 0 && desiredStates.get(0).vec().distTo(robot.getPos().vec()) < 0.5){
                robot.drive.setPower(0, 0, 0);
                desiredStates.remove(0);
                state++;
                paused = true;
            }else if(paused){
                if(profiler.getDeltaTime(TimeUnits.SECONDS, false) > 1){
                    paused = false;
                }else{
                    robot.drive.setPower(0, 0, 0);
                }
            }else{
                double max_pow = 1.0;
                if(state == 3 && robot.getPos().getY() > 23 && robot.getPos().getY() < 53){
                    max_pow = 0.4;
                }

                profiler.reset();

                if(desiredStates.size() != 0 && robot.getPos().vec().distTo(desiredStates.get(0).vec()) < 5){
                    telemetry.addData("...", "In PID kickout");
                    robot.GoTo(desiredStates.get(0), new Pose2d(1.0, 1.0, 1.0));
                }else{
                    telemetry.addData("...", "In MPC");
                    robot.drive.goToPointFF(new Pose2d(-positions.get(timestepIndex).getY(), positions.get(timestepIndex).getX(), -positions.get(timestepIndex).getHeading()),
                            robot.getPos(),
                            max_pow,
                            motorActuations.get(timestepIndex).get(0),
                            motorActuations.get(timestepIndex).get(2),
                            motorActuations.get(timestepIndex).get(1),
                            motorActuations.get(timestepIndex).get(3));
                }
            }

            telemetry.addData("State", state);
            telemetry.addData("Position", robot.getPos());
            telemetry.addData("Timestamp", timesteps.size());
            telemetry.addData("Positions", positions.size());
            telemetry.addData("Motor Actuation", motorActuations.size());
            prevIndex = timestepIndex;

            telemetry.update();
            robot.drive.write();
        }
    }

    public int posSearch(Pose2d pose, ArrayList<Pose2d> a) {
        int closest = 0;

        for(int i = 0; i < a.size(); i++){
            if(Math.abs(i - prevIndex) < 20/*Math.abs(timesteps.get(i) - profiler.getDeltaTime(false)) < 100 */&&
                    pose.vec().distTo(a.get(i).vec()) < pose.vec().distTo(a.get(closest).vec())/* &&
                    Math.abs(pose.heading - a.get(i).heading) < Math.abs(pose.heading - a.get(closest).heading)*/){
                closest = i;
            }
        }

        return closest;
    }

    public static int timeStepSearch(double value, ArrayList<Integer> a) {
        if(value < a.get(0)) {
            return 0;
        }
        if(value > a.get(a.size() - 1)) {
            return a.size() - 1;
        }

        int lo = 0;
        int hi = a.size() - 1;

        while (lo <= hi) {
            int mid = (hi + lo) / 2;

            if (value < a.get(mid)) {
                hi = mid - 1;
            } else if (value > a.get(mid)) {
                lo = mid + 1;
            } else {
                return mid;
            }
        }

        // lo == hi + 1
        return (a.get(lo) - value) < (value - a.get(hi)) ? lo : hi;
    }
}
