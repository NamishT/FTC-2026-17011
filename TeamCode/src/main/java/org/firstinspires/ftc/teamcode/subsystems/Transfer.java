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
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;

@Config
@TeleOp
public class Transfer extends OpMode
{
    private CRServo transferServo;

    @Override
    public void init()
    {
        transferServo = hardwareMap.get(CRServo.class, "transfer");
        transferServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop()
    {

    }

    public void setPower(double power){
        if(power>1.0)power=1.0;
        if(power<-1.0)power=-1.0;
        transferServo.setPower(power);
    }
    public double getPower(){
        return transferServo.getPower();
    }

}