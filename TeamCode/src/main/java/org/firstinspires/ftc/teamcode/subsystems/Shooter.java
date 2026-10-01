package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Review;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class Shooter extends OpMode {

    private double velocityN = 1000;
    private double velocityP = 1000;
    private static DcMotorEx shooterP;
    private static DcMotorEx shooterN;

    private Intake intake;
    private Transfer transfer;

    public enum SHOOTER_STATES{
        INACTIVE,
        SPIN_UP,
        SHOOT,
        END,

    }
    public SHOOTER_STATES currentState = SHOOTER_STATES.INACTIVE;

    @Override
    public void init(){
        shooterP = hardwareMap.get(DcMotorEx.class, "shooterP");
        shooterN= hardwareMap.get(DcMotorEx.class, "shooterN");

        shooterP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterP.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterN.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){

    }
    public void shooting_machine(){
        switch(currentState) {
            case INACTIVE:
                shooterN.setPower(0);
                shooterP.setPower(0);
                intake.setPower(0);
                transfer.setPower(0);
                break;
            case SPIN_UP:
                shooterN.setVelocity(velocityN);
                shooterP.setVelocity(velocityP);
                if(shooterN.getVelocity()==velocityN&&shooterP.getVelocity()==velocityP){
                    currentState = SHOOTER_STATES.SHOOT;
                }
                break;
            case SHOOT:
                intake.setPower(1);
                transfer.setPower(1.0);
                if(gamepad1.right_trigger<.3){
                    currentState = SHOOTER_STATES.END;
                }
                break;
            case END:
                shooterN.setPower(0);
                shooterP.setPower(0);
                intake.setPower(0);
                transfer.setPower(0);
        }
    }
}
