package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake{
    private static DcMotorEx rightIntake;
    private static DcMotorEx leftIntake;
    // 0.5 = closed 0 = open
    public static CRServo gateP;
    public static Servo gateN;
    public static CRServo blockN;
    private static CRServo flushPL;
    private static CRServo flushPR;
    public Intake(HardwareMap hardwareMap){
        rightIntake = hardwareMap.get(DcMotorEx.class, "rightIntake");
        leftIntake = hardwareMap.get(DcMotorEx.class, "leftIntake");

        gateP = hardwareMap.get(CRServo.class, "gateP");
        gateN = hardwareMap.get(Servo.class, "gateN");
        blockN = hardwareMap.get(CRServo.class, "blockN");

        flushPL = hardwareMap.get(CRServo.class, "flushPL");
        flushPR = hardwareMap.get(CRServo.class, "flushPR");

        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power){


        power = Math.max(-1.0, Math.min(1.0, power));

        rightIntake.setPower(power);
        leftIntake.setPower(power);
    }
    public void setPowerGateOpen() {
        gateP.setPower(0);
        gateN.setPosition(0);
        blockN.setPower(1);
    }
    public void setPowerGateClose(){
        gateP.setPower(0);
        gateN.setPosition(0.5);
        blockN.setPower(-1);
    }
    public void openFlush(){
        flushPL.setPower(-1);
        flushPR.setPower(-1);
    }
    public void closeFlush(){
        flushPL.setPower(1);
        flushPR.setPower(1);
    }
    public double getRightIntakePower(){
        return rightIntake.getPower();
    }
    public double getLeftIntakePower(){
        return leftIntake.getPower();
    }

    public double getVelocity(){
        return rightIntake.getVelocity();
    }
}
// intake + transfer
//  servos on intake transfer
// coding swerving of the CR servos
// limelight with apriltag facing