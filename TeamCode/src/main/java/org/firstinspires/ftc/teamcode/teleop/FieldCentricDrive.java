package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.arcrobotics.ftclib.controller.PIDController;
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
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.universalconstants;


import java.util.List;

@TeleOp(name = "TeleOp", group = "OpMode")
public class FieldCentricDrive extends OpMode {
    private Timer lastPressTimer;
    Boolean autoSort = true;
    public static double x, y, rx;


    @Override
    public void init(){
        Turret.init(hardwareMap, telemetry);
        Launcher.init(hardwareMap);
        DriveTrain.init(hardwareMap);
        Intake.init(hardwareMap);

        lastPressTimer = new Timer();
    }

    @Override
    public void loop() {
        Turret.runTurret(telemetry);
        y = gamepad1.left_stick_x; // Remember, this is reversed
        x = -gamepad1.left_stick_y; // Counteract imperfect strafing
        rx = gamepad1.right_stick_x * .75;
        DriveTrain.fieldCentricDrive();

        if (gamepad1.right_bumper){
            Launcher.openLauncher();
            Intake.motorIntake.setPower(1);
        } else {
            Launcher.closeLauncher();
        }

        if(lastPressTimer.getElapsedTimeSeconds() > universalconstants.lastPressTime){
            if (gamepad1.x) {
                lastPressTimer.resetTimer();
                Intake.runIntake();
            }
            if (gamepad1.b){
                lastPressTimer.resetTimer();
                Intake.reverseIntake();
            }
            if (gamepad1.y) {
                Launcher.runLauncher();
                lastPressTimer.resetTimer();
            }
//
        }

        Launcher.setShooterVelo();
        double curVelocity = Launcher.motorShooterRight.getVelocity();
        double error = Launcher.curTargetVelo - curVelocity;

        telemetry.addData("Target Velocityfjdksla;fdsa:", Launcher.curTargetVelo)
                .addData("Current Velocity", curVelocity)
                .addData("Error", error);
        telemetry.update();
    }
    @Override
    public void stop() {
        super.stop();
    }
}
