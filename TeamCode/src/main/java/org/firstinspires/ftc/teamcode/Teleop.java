package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Intake;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.pedropathing.revhub.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.pedro.Constants;


@Config
@TeleOp(name = "Teleop")
public class Teleop extends OpMode {


    private Drivebase drivebase;
    private Shooter shooter;
    private Intake intake;


    private PinpointLocalizer localizer;


    private GoBildaPinpointDriver pinpoint;


    private FtcDashboard dash;
    private Telemetry myTe;


    public static double velocityNTarget = 1100;
    public static double velocityPTarget = 3000;


    // 0 = Blue, 1 = Red
    private int alliance = 1;


    @Override
    public void init() {


        localizer = new PinpointLocalizer(
                hardwareMap,
                Constants.localizerConfig
        );


        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );


        dash = FtcDashboard.getInstance();


        myTe = new MultipleTelemetry(
                dash.getTelemetry(),
                telemetry
        );


        intake = new Intake(hardwareMap);
        drivebase = new Drivebase(hardwareMap);
        shooter = new Shooter(hardwareMap);


        myTe.addLine("TeleOp Initialized");
        myTe.update();
    }


    @Override
    public void loop() {


        // Alliance selection
        /*
        if (gamepad1.xWasPressed()) {

            alliance = alliance == 1 ? 0 : 1;

            drivebase.resetAim();
        }
        */


        if(gamepad2.dpad_up) {
            intake.setPowerGateOpen();
        }
        if(gamepad2.dpad_down) {
            intake.setPowerGateClose();
        }
        if(gamepad2.dpad_left) {
            intake.openFlush();
        }
        if(gamepad2.dpad_right) {
            intake.closeFlush();
        }



        /*
        boolean aimRequested =
                gamepad1.right_trigger > 0.3 ||
                        gamepad2.right_trigger > 0.3;


        boolean reverseRequested =
                gamepad1.left_trigger > 0.3 ||
                        gamepad2.left_trigger > 0.3;


        if (!aimRequested) {
            drivebase.resetAim();
        }


        if (aimRequested || reverseRequested) {

            drivebase.stopRobot();

        } else {
        */


        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;


        drivebase.drive(drive, strafe, turn);

        // }


        // Shooter, intake, and transfer
        shooting_machine(false, false);


        // Update localization
        localizer.update();
        pinpoint.update();


        double heading = pinpoint.getPosition()
                .getHeading(AngleUnit.DEGREES);


        // Telemetry
        /*
        myTe.addData("Alliance",
                alliance == 1 ? "RED" : "BLUE");


        myTe.addData("AprilTag Found",
                drivebase.hasTarget());


        myTe.addData("Camera Valid",
                drivebase.isCameraValid());


        myTe.addData("AprilTag TX",
                drivebase.getTargetTX());


        myTe.addData("Selected Tag ID",
                drivebase.getSelectedTagID());


        myTe.addData("Aligned",
                drivebase.isAligned());
        */


        myTe.addData("Shooter Velocity",
                shooter.getVelocityP());


        myTe.addData("Target Velocity",
                velocityPTarget);


        myTe.addData("X",
                localizer.pose().x());


        myTe.addData("Y",
                localizer.pose().y());


        myTe.addData("Heading",
                heading);


        myTe.update();
    }


    public void shooting_machine(boolean aimRequested, boolean reverseRequested) {


        /*
        // Reverse controls
        if (reverseRequested) {

            shooter.setPower(-1.0);
            intake.setPower(1.0);

            return;
        }
        */


        // Normal shooter velocity control
        if (shooter.getVelocityP() <
                velocityPTarget * 0.98) {
            shooter.setVelocityP(velocityPTarget);
        }
        if(shooter.getVelocityN() < velocityNTarget * 0.98) {
            shooter.setVelocityN(velocityNTarget);
        }
        else {


            shooter.setVelocityP(velocityPTarget);
        }


        /*
        // Automatic AprilTag aiming
        if (aimRequested) {

            if (shooter.getVelocityP() >=
                    velocityPTarget * 0.98) {
                boolean aligned =
                        drivebase.turnToAprilTag(alliance);
                if (aligned &&
                        drivebase.hasTarget()) {
                } else {
                }
            } else {
                drivebase.stopRobot();
            }

            intake.setPower(0);
            return;
        }
        */


        // Manual intake
        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            intake.setPower(-1.0);
        }
        else {
            intake.setPower(0);
        }
    }


    @Override
    public void stop() {


        drivebase.stopRobot();


        intake.setPower(0);
        shooter.setPower(0);
    }
}