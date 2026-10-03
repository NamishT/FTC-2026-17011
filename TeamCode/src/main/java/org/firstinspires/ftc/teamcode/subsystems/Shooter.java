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
    


    public Shooter(HardwareMap hardwareMap){
        shooterP = hardwareMap.get(DcMotorEx.class, "shooterP");
        shooterN= hardwareMap.get(DcMotorEx.class, "shooterN");

        shooterP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterP.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterN.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power){
        if(power>1.0)power=1.0;
        if(power<-1.0) power = -1.0;
        shooterP.setPower(power);
        shooterN.setPower(power);

    }
    public void setVelocity(double velocity){
        shooterP.setVelocity(velocity);
        shooterN.setVelocity(velocity);
    }

    public double getVelocityN(){
        return shooterN.getVelocity();
    }
    public double getVelocityP(){
        return shooterP.getVelocity();
    }



}
