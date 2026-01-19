package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {
    public static DcMotorEx motorShooterRight, motorShooterLeft;
    public static Servo shooterServo;
    public static double curTargetVelo;
    public static double P=300,I=0,D=0,F=14.25;

    public static void init(HardwareMap hardwareMap) {
        motorShooterLeft = hardwareMap.get(DcMotorEx.class,"m6");
        motorShooterRight = hardwareMap.get(DcMotorEx.class, "m7");
        shooterServo = hardwareMap.servo.get("s0");

        motorShooterLeft.setDirection(DcMotor.Direction.REVERSE);
        motorShooterRight.setDirection(DcMotor.Direction.FORWARD);
        motorShooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients PIDFCoefficients = new PIDFCoefficients(P, I, D, F);
        motorShooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDFCoefficients);

        curTargetVelo = 0;

        shooterServo.setPosition(universalconstants.shooterServoClose);
    }
    public static void setShooterVelo(){
        motorShooterRight.setVelocity(curTargetVelo);
        motorShooterLeft.setPower(motorShooterRight.getPower());
    }
    public static void runLauncher() {
        if (curTargetVelo == universalconstants.shooterVeloReady) {
            curTargetVelo = universalconstants.shooterVeloIdle;
        } else {
            curTargetVelo = universalconstants.shooterVeloReady;
        }
    }
    public static void openLauncher(){
        shooterServo.setPosition(universalconstants.shooterServoOpen);
    }
    public static void closeLauncher(){
        shooterServo.setPosition(universalconstants.shooterServoClose);
    }
}