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
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@TeleOp

public class Teleop extends OpMode {
    Drivebase drivebase;
    Transfer transfer;
    Shooter shooter;
    Intake intake;

    PinpointLocalizer localizer;

    private double velocityN = 1100;
    private double velocityP = 1100;

    private GoBildaPinpointDriver pinpoint;
    //private Limelight3A limelight;

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
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
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

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class, "pinpoint"
        );

//        limelight = hardwareMap.get(
//                Limelight3A.class, "limelight"
//        );
//
//        limelight.pipelineSwitch(0);
//        limelight.start();

        localizer.update();

        myTe.addData("X", localizer.pose().x());
        myTe.addData("Y", localizer.pose().y());
        telemetry.update();

        pinpoint.update();

        double heading = pinpoint.getPosition()
                .getHeading(AngleUnit.DEGREES);

//        LLResult result = limelight.getLatestResult();
//
//        if (result != null && result.isValid()
//                && !result.getFiducialResults().isEmpty()) {
//
//            LLResultTypes.FiducialResult tag =
//                    result.getFiducialResults().get(0);
//
//            double targetX = tag.getTargetXDegrees();
//
//            myTe.addData("AprilTag ID", tag.getFiducialId());
//            myTe.addData("Target X", targetX);
//        } else {
//            myTe.addData("AprilTag", "Not detected");
//        }
//
//        myTe.addData("Pinpoint Heading", heading);
//        myTe.update();
//
//        if (gamepad1.a) {
//
//
//            boolean aimed = drivebase.aimAtAprilTag(result);
//
//            telemetry.addData("Aimed", aimed);
//        } else {
//            // Normal manual driving
//            drivebase.drive(
//                    -gamepad1.left_stick_y,
//                    gamepad1.left_stick_x,
//                    gamepad1.right_stick_x
//            );
//        }
//
//        telemetry.addData("Pinpoint Heading", heading);
//        telemetry.update();

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
                //shooter.setVelocityN(velocityN);

                if(shooter.getVelocityN()>=.98*velocityN||shooter.getVelocityP()>=.98*velocityP){
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
