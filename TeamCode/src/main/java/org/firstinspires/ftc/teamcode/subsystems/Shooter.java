package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Review;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Shooter {


    private static DcMotorEx shooterP;
    private static DcMotorEx shooterN;

    public static double P = 150;
    public static double I = 0;
    public static double D = .5;
    public static double F = 15;

    public Shooter(HardwareMap hardwareMap){


        shooterP = hardwareMap.get(DcMotorEx.class, "shooterP");
        shooterN= hardwareMap.get(DcMotorEx.class, "shooterN");

        shooterP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterP.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterN.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterP.setVelocityPIDFCoefficients(P, I, D, F);
        shooterN.setVelocityPIDFCoefficients(P, I, D, F);
    }

    public void setPower(double power){
        if(power>1.0)power=1.0;
        if(power<-1.0) power = -1.0;
        shooterP.setPower(power);
        shooterN.setPower(power);

    }
    public void setVelocityP(double velocity){
        shooterP.setVelocity(velocity);
    }

    public void setVelocityN(double velocity){
        shooterN.setVelocity(velocity);
    }


    public double getVelocityN(){
        return shooterN.getVelocity();
    }
    public double getVelocityP(){
        return shooterP.getVelocity();
    }



}
