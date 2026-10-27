package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
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