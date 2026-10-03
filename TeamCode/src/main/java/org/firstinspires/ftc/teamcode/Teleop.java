package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp

public class Teleop extends OpMode {
    Drivebase drivebase;
    Transfer transfer;
    Shooter shooter;
    Intake intake;

    private double velocityN = 1100;
    private double velocityP = 1100;

    public enum SHOOTER_STATES{
        INACTIVE,
        SPIN_UP,
        SHOOT,

    }

    FtcDashboard dash;
    private static Telemetry myTe;

    public SHOOTER_STATES currentState = SHOOTER_STATES.INACTIVE;
    @Override
    public void init(){
        dash = FtcDashboard.getInstance();
        myTe = new MultipleTelemetry(dash.getTelemetry(), telemetry);

        intake = new Intake(hardwareMap);
        drivebase = new Drivebase(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        myTe.update();
    }

    @Override
    public void loop(){
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        drivebase.drive(drive, strafe, turn);
        shooting_machine();
        myTe.addData("Pollen Velocity", shooter.getVelocityP());
        myTe.addData("Nectar Velocity", shooter.getVelocityN());
        myTe.update();

    }
    public void shooting_machine(){
        switch(currentState) {
            case INACTIVE:


                intake.setPower(0);
                transfer.setPower(0);
                if(gamepad1.right_trigger_pressed){
                    currentState= SHOOTER_STATES.SPIN_UP;
                }
                if(gamepad1.left_trigger_pressed){
                    shooter.setPower(-1);

                    intake.setPower(1);
                    transfer.setPower(1);

                }
                if(gamepad1.right_bumper){
                    intake.setPower(-1);
                }
                else{
                    shooter.setPower(.2);
                }
                break;
            case SPIN_UP:
                shooter.setVelocityP(velocityP);
                shooter.setVelocityN(velocityN);

                if(shooter.getVelocityN()>=.98*velocityN&&shooter.getVelocityP()>=.98*velocityP){
                    currentState = SHOOTER_STATES.SHOOT;
                }

                if(gamepad1.right_trigger<.3){
                    currentState = SHOOTER_STATES.INACTIVE;
                }
                if(gamepad1.left_trigger_pressed){
                    shooter.setPower(-1);
                    intake.setPower(1);
                    transfer.setPower(1);
                    currentState = SHOOTER_STATES.INACTIVE;
                }
                break;
            case SHOOT:
                transfer.setPower(-1.0);
                if(gamepad1.right_trigger<.3){
                    currentState = SHOOTER_STATES.INACTIVE;
                }
                if(gamepad1.left_trigger_pressed){
                    shooter.setPower(-1);
                    intake.setPower(1);
                    transfer.setPower(1);
                    currentState = SHOOTER_STATES.INACTIVE;
                }
                break;

        }
    }

}
