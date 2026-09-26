package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp
public class Review extends OpMode {

    public enum SHOOTER_STATES{
        INACTIVE,
        SPIN_UP,
        SHOOT,
        END
    }

    public SHOOTER_STATES currentState = SHOOTER_STATES.INACTIVE;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight, intake, shooter, transfer;

    private FtcDashboard dash;
    private Servo gate;

    private static Telemetry myTe;
    private IMU imu;
    public static double shooterVelocity = 1700;
    public static double closeRangev = 1700;
    public static double longRangev = 1900;
    public static double gatePosition = 0.25;
    public static double gateClose = 0.4;

    @Override
    public void init(){
        dash = FtcDashboard.getInstance();
        myTe = new MultipleTelemetry(dash.getTelemetry(), telemetry);
        initializeMotors();
        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
        myTe.update();
        imu.resetYaw();

    }
    @Override
    public void loop() {
        mecanumDrive();
        mechanismCode();
        shootingMachine();

        myTe.addData("Front Right Velocity", frontRight.getVelocity());
        myTe.addData("Front Left Velocity", frontLeft.getVelocity());
        myTe.addData("Back Right Velocity", backRight.getVelocity());
        myTe.addData("Back Left Velocity", backLeft.getVelocity());
        myTe.update();

    }

    public void initializeMotors(){
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        transfer = hardwareMap.get(DcMotorEx.class, "transferMotor");
        transfer.setDirection(DcMotorEx.Direction.REVERSE);


        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        gate = hardwareMap.get(Servo.class, "transferGate");
    }

    public void mecanumDrive(){
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) +Math.abs(turn), 1.0);
        double frontLeftPower = (drive+strafe+turn)/denominator;
        double backLeftPower = (drive-strafe+turn)/denominator;
        double frontRightPower = (drive-strafe-turn)/denominator;
        double backRightPower = (drive+strafe-turn)/denominator;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }
    public void fieldOrientedDrive(){
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double rotX = strafe* Math.cos(botHeading) - drive*Math.sin(botHeading);
        double rotY = strafe* Math.sin(botHeading) + drive*Math.cos(botHeading);

        double denominator = Math.max(Math.abs(rotY)+Math.abs(rotX)+Math.abs(turn), 1.0);
        double frontLeftPower = (rotY - rotX + turn)/denominator;
        double backLeftPower = (rotY+rotX+turn)/denominator;
        double frontRightPower = (rotY+rotX-turn)/denominator;
        double backRightPower = (rotY-rotX-turn)/denominator;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void mechanismCode() {
        if(gamepad1.right_bumper){
            intake.setPower(1.0);
            transfer.setPower(1);
        }
        else if(gamepad1.left_bumper) {
            intake.setPower(-1);
            transfer.setPower(-1);
            shooter.setPower(-1);
        }
        else if(!currentState.equals(SHOOTER_STATES.INACTIVE)){

        }
        else{
            intake.setPower(0);
            transfer.setPower(0);
            shooter.setPower(0);
                }

        if(gamepad1.triangleWasPressed() && shooterVelocity < 1800){
            shooterVelocity = longRangev;
        }
        else if(gamepad1.triangleWasPressed() && shooterVelocity >1800){
            shooterVelocity = closeRangev;
        }
        telemetry.addData("Shooter Velocity:", shooter.getVelocity());
        if(gamepad1.right_trigger > .3 && currentState.equals(SHOOTER_STATES.INACTIVE)){
            currentState = SHOOTER_STATES.SPIN_UP;
        }
    }

    public void shootingMachine(){
        switch (currentState) {
            case INACTIVE:
                gate.setPosition(gateClose);
                break;
            case SPIN_UP:
                gate.setPosition(gateClose);
                shooter.setVelocity(shooterVelocity);

                if(shooter.getVelocity()>shooterVelocity*.98){
                    currentState = SHOOTER_STATES.SHOOT;
                }
                break;
            case SHOOT:
                gate.setPosition(gatePosition);
                transfer.setPower(1);
                if(gamepad1.right_trigger<.3){
                    currentState = SHOOTER_STATES.END;
                }
                break;
            case END:
                shooter.setPower(0);
                transfer.setPower(0);
                gate.setPosition(gateClose);
                intake.setPower(0);
                currentState = SHOOTER_STATES.INACTIVE;


        }
    }
}
