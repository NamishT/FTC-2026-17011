package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class Intake extends OpMode {
    private static DcMotorEx intake;
    public void init(){
        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
    }
    public void loop(){


    }

    public void setPower(double power){
        if(power>1.0)power=1.0;
        if(power<-1.0) power = -1.0;
        intake.setPower(power);
    }
    public double getPower(){
        return intake.getPower();
    }

    public double getVelocity(){
        return intake.getVelocity();
    }

}