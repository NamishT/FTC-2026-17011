package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp

public class Teleop extends OpMode {
    Drivebase drivebase = new Drivebase(hardwareMap);
    Transfer transfer = new Transfer(hardwareMap);
    Shooter shooter = new Shooter(hardwareMap);
    Intake intake = new Intake(hardwareMap);

    private double velocityN = 1000;
    private double velocityP = 1000;

    public enum SHOOTER_STATES{
        INACTIVE,
        SPIN_UP,
        SHOOT,

    }
    public SHOOTER_STATES currentState = SHOOTER_STATES.INACTIVE;
    @Override
    public void init(){

    }

    @Override
    public void loop(){
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        drivebase.drive(drive, strafe, turn);
        shooting_machine();

    }
    public void shooting_machine(){
        switch(currentState) {
            case INACTIVE:
                shooter.setPower(0);
                shooter.setPower(0);
                intake.setPower(0);
                transfer.setPower(0);
                if(gamepad1.right_trigger_pressed){
                    currentState= SHOOTER_STATES.SPIN_UP;
                }
                if(gamepad1.left_trigger_pressed){
                    shooter.setPower(-1);
                    shooter.setPower(-1);
                    intake.setPower(-1);
                    transfer.setPower(-1);

                }
                break;
            case SPIN_UP:
                shooter.setVelocity(velocityN);
                shooter.setVelocity(velocityP);
                if(shooter.getVelocityN()==velocityN&&shooter.getVelocityP()==velocityP){
                    currentState = SHOOTER_STATES.SHOOT;
                }
                if(gamepad1.right_trigger<.3){
                    currentState = SHOOTER_STATES.INACTIVE;
                }
                if(gamepad1.left_trigger_pressed){
                    shooter.setPower(-1);
                    shooter.setPower(-1);
                    intake.setPower(-1);
                    transfer.setPower(-1);
                    currentState = SHOOTER_STATES.INACTIVE;
                }
                break;
            case SHOOT:
                intake.setPower(1);
                transfer.setPower(1.0);
                if(gamepad1.right_trigger<.3){
                    currentState = SHOOTER_STATES.INACTIVE;
                }
                if(gamepad1.left_trigger_pressed){
                    shooter.setPower(-1);
                    shooter.setPower(-1);
                    intake.setPower(-1);
                    transfer.setPower(-1);
                    currentState = SHOOTER_STATES.INACTIVE;
                }
                break;

        }
    }

}
