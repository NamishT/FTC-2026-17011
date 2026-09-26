package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp
public class ShooterTest extends OpMode{

    public static FtcDashboard dash;
    private static Telemetry myTe;

    private DcMotorEx shooterN, shooterP;
    private CRServo servoP, transferServo;
    private CRServo servoN;

    public static double shooterNVelocity;
    public static double shooterPVelocity;



    @Override
    public void init(){
        shooterP = hardwareMap.get(DcMotorEx.class, "shooterP");
        shooterN = hardwareMap.get(DcMotorEx.class, "shooterN");
        servoP = hardwareMap.get(CRServo.class, "servoP");
        servoN = hardwareMap.get(CRServo.class, "servoN");
        dash = FtcDashboard.getInstance();
        myTe = new MultipleTelemetry(dash.getTelemetry(), telemetry);
//        transferServo = hardwareMap.get(CRServo.class, "transferServo");
        shooterP.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterN.setDirection(DcMotorSimple.Direction.REVERSE);
        myTe.update();
        shooterPVelocity = 1000 ;
        shooterNVelocity = 1250;
    }

    @Override
    public void loop(){
        servoN.setPower(0);
        servoP.setPower(0);
        shooterP.setPower(0);
        shooterN.setPower(0);
//        transferServo.setPower(0);
        if(gamepad1.left_bumper) {
            servoP.setPower(-1);
            shooterP.setVelocity(shooterPVelocity);
//            if(shooterP.getVelocity()>=.95*shooterPVelocity)
//                servoP.setPower(1);
//            else{
//                servoP.setPower(0);
//            }
        }
        if(gamepad1.right_bumper) {
            servoN.setPower(1);
            shooterN.setVelocity(shooterNVelocity);
//            if(shooterN.getVelocity()>=.95*shooterNVelocity)
//                servoN.setPower(1);
//            else{
//                servoP.setPower(0);
//            }

        }

        myTe.addData("shooterN Speed", shooterP.getVelocity()*28/60);
        myTe.addData("shooterP Speed", shooterP.getVelocity()*28/60);
        myTe.addData("shooterN Speed given", shooterNVelocity);
        myTe.addData("shooterP Speed given", shooterPVelocity);


        myTe.update();
    }
}
