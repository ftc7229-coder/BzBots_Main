package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public static DcMotor motorIntake;
    public static void init(HardwareMap hardwareMap){
        motorIntake = hardwareMap.dcMotor.get("m5");
        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public static void runIntake(){
        if (motorIntake.getPower() < .25) {
            motorIntake.setPower(universalconstants.intakePowerActive);
        } else {
            motorIntake.setPower(universalconstants.intakePowerStop);
        }
    }
    public static void reverseIntake(){
        if (motorIntake.getPower() > -0.1) {
            motorIntake.setPower(universalconstants.intakePowerReverse);
        } else {
            motorIntake.setPower(universalconstants.intakePowerStop);
        }
    }
}
