package org.firstinspires.ftc.teamcode.subsystems;

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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;


public class Transfer{
    private CRServo transferN, transferP;


    public Transfer(HardwareMap hardwareMap){
        transferN = hardwareMap.get(CRServo.class, "transferN");
        transferP = hardwareMap.get(CRServo.class, "transferP");

        transferP.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void setPower(double power){
        if(power>1.0)power=1.0;
        if(power<-1.0)power=-1.0;
        transferP.setPower(power);
        transferN.setPower(power);

    }
    public double getTransferNPower() {
        return transferN.getPower();
    }

    public double getTransferPPower(){
        return transferP.getPower();
    }



}