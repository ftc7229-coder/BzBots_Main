package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

@TeleOp
public class Turret extends OpMode {
    private Timer lastPressTimer;
    public static double x, y, rx;
    @Override
    public void init() {
        org.firstinspires.ftc.teamcode.subsystems.Turret.init(hardwareMap, telemetry);
        Launcher.init(hardwareMap);
        DriveTrain.init(hardwareMap);
        Intake.init(hardwareMap);

        lastPressTimer = new Timer();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        org.firstinspires.ftc.teamcode.subsystems.Turret.runTurret(telemetry);
        y = -gamepad1.left_stick_x; // Remember, this is reversed
        x = gamepad1.left_stick_y; // Counteract imperfect strafing
        rx = -gamepad1.right_stick_x * .75;
        //DriveTrain.fieldCentricDrive(DriveTrain.odo);
    }

    @Override
    public void stop() {
        super.stop();
    }
}
