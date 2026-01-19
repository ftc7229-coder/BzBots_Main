package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Turret.follower;
import static org.firstinspires.ftc.teamcode.teleop.FieldCentricDrive.rx;
import static org.firstinspires.ftc.teamcode.teleop.FieldCentricDrive.x;
import static org.firstinspires.ftc.teamcode.teleop.FieldCentricDrive.y;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DriveTrain {
    public static DcMotor motorBackRight, motorBackLeft, motorFrontRight, motorFrontLeft;
    public static GoBildaPinpointDriver odo;
    public static void init(HardwareMap hardwareMap){
        motorFrontLeft = hardwareMap.dcMotor.get("m0");
        motorFrontRight = hardwareMap.dcMotor.get("m1");
        motorBackRight = hardwareMap.dcMotor.get("m2");
        motorBackLeft = hardwareMap.dcMotor.get("m3");


        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public static void fieldCentricDrive(){
//        if (gamepad1.left_bumper){
//            wheelspeed = .5;
//            turnspeed = .4;
//        }
// Read inverse IMU heading, as the IMU heading is CW positive
        Pose roboPose = follower.getPose();
        double botHeading = roboPose.getHeading();
        double rotY = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotX = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;

        motorFrontLeft.setPower(frontLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);
        motorBackLeft.setPower(backLeftPower);
    }
}
