package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake{
    private static DcMotorEx intake ;
    private static CRServo leftIntake, rightIntake;


    public Intake(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        leftIntake = hardwareMap.get(CRServo.class, "miniIntakeWheel1");
        rightIntake = hardwareMap.get(CRServo.class, "miniIntakeWheel2");

        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power){
        if(power>1.0)power=1.0;
        if(power<-1.0) power = -1.0;
        intake.setPower(power);
        leftIntake.setPower(power);
        rightIntake.setPower(power);
    }
    public double getMainIntakePower(){
        return intake.getPower();
    }
    public double getRightIntakePower(){
        return rightIntake.getPower();
    }
    public double getLeftIntakePower(){
        return leftIntake.getPower();
    }

    public double getVelocity(){
        return intake.getVelocity();
    }

}