package org.firstinspires.ftc.teamcode.Tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;


@Configurable
@TeleOp
public class TurretTest extends OpMode {
    PIDController controller;
    PIDController controller2;

    public static double p = 0.035, i = 0, d = 0.001;
    public static double defult_f = -0.2;
    public static double f = defult_f;

    public static double p2 = 0.015, i2 = 0, d2 = 0.001;
    public static double defult_f2 = -0.2;
    public static double f2 = defult_f2;


    static Limelight3A limelight;
    public int TurretState = 1;
    DcMotor turretMotor;
    public double goalOffset;

    public double goalPosY = 136;
    public double goalPosX = 140;
    public double encododersPerDegree = 60/27 /*gear ratio*/ * 537.7/*Encoders per rotation*/ / 360;

    Follower follower;
    public static final Pose startPose = new Pose(124, 121, Math.toRadians(90));

    public double maxDegreeRange = 95;
    Pose roboPose;

    public boolean usingCamera = false;
    public double acceptableDegrees = 1;
    public double cameraBufferTime = .2;

    Timer cameraTimer;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        controller2 = new PIDController(p2, i2, d2);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        turretMotor = hardwareMap.dcMotor.get("m4");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        controller.setPID(p, i, d);
        controller2.setPID(p2, i2, d2);

        cameraTimer = new Timer();
    }

    @Override
    public void loop() {
        follower.update();


        if (gamepad1.a && TurretState != 1){
            telemetry.addData("Turret Status: ", "ON");
            TurretState = 1;
        } else if (gamepad1.a && TurretState == 1) {
            telemetry.addData("Turret Status:", "OFF");
            TurretState = 0;
        }

        if (gamepad1.b && TurretState != 2){
            TurretState = 2;
        } else if (gamepad1.b && TurretState == 2) {
            TurretState = 0;
        }

        if (gamepad1.x){
            TurretState = 0;
        }

        if (TurretState == 1){
            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                telemetry.addData("Result Status", "Is Valid");
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                usingCamera = true;
                cameraTimer.resetTimer();
                for (LLResultTypes.FiducialResult tag : tags) {
                    double angleX = tag.getTargetXDegrees();
                    telemetry.addData("Tag " + tag.getFiducialId() + " X°", angleX);

                    goalOffset = angleX;

                    if(turret_angle_to_robot() - get_goal_angle() < 0){
                        f2 = -defult_f2;
                    } else if (turret_angle_to_robot() - get_goal_angle() > 0) {
                        f2 = defult_f2;
                    }
                    if (Math.abs(turret_angle_to_robot() - get_goal_angle()) < acceptableDegrees) {
                        f2 = 0;
                    }


                    if (Math.abs(get_goal_angle()) < maxDegreeRange) {
                        double pid = controller.calculate(0, goalOffset);
                        double power = pid + f2;
                        turretMotor.setPower(power);
                    } else {
                        turretMotor.setPower(0);
                    }

                }


            } else if (cameraTimer.getElapsedTimeSeconds() > cameraBufferTime){
                usingCamera = false;
                if(turret_angle_to_robot() - get_goal_angle() < 0){
                    f = -defult_f;
                } else if (turret_angle_to_robot() - get_goal_angle() > 0) {
                    f = defult_f;
                }
                if (Math.abs(turret_angle_to_robot() - get_goal_angle()) < acceptableDegrees) {
                    f = 0;
                }


                if (Math.abs(get_goal_angle()) < maxDegreeRange) {
                    double pid = controller.calculate(turret_angle_to_robot(), get_goal_angle());
                    double power = pid + f;
                    //turretMotor.setPower(power);
                } else {
                    turretMotor.setPower(0);
                }
            } else {
                turretMotor.setPower(0);
            }


        } else if (TurretState == 2) {
            double pid = controller.calculate(turret_angle_to_robot(), 0);
            double power = pid + f;
            turretMotor.setPower(power);


        } else{
            turretMotor.setPower(0);
        }



        telemetry.addData("Motor Power:", turretMotor.getPower());
        telemetry.addData("feedF", f);
        telemetry.addData("Motor Position", turretMotor.getCurrentPosition());
        telemetry.addData("Distance to Goal", get_goal_distance());
        telemetry.addData("Angle to Goal", get_goal_angle());
        telemetry.addData("Heading", roboPose.getHeading());
        telemetry.addData("turret angle", turret_angle_to_robot());
        telemetry.addData("unmod turret angle", turretMotor.getCurrentPosition());
        telemetry.addData("turret degrees offset", get_goal_angle() - turret_angle_to_robot());
        telemetry.addData("Using camera", usingCamera);
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
    }

    public double get_goal_distance() {
        roboPose = follower.getPose();

        return  Math.abs(Math.sqrt(Math.pow(roboPose.getX() - goalPosX, 2) + Math.pow(roboPose.getY() - goalPosY, 2)));
    }

    public double get_goal_angle() {
        roboPose = follower.getPose();
        return Math.toDegrees(roboPose.getHeading() - Math.toRadians(90) + Math.atan((roboPose.getX() - goalPosX)/(roboPose.getY() - goalPosY)));
    }

    public double turret_angle_to_robot() {
        return turretMotor.getCurrentPosition() / encododersPerDegree;
    }
}
