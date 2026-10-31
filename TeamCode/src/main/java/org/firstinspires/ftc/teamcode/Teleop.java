
package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;


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


    Drivebase drivebase;
    Transfer transfer;
    Shooter shooter;
    Intake intake;


    PinpointLocalizer localizer;
    GoBildaPinpointDriver pinpoint;


    FtcDashboard dash;
    private Telemetry myTe;


    private double velocityPTarget = 1100;


    // 0 = Blue, 1 = Red
    private int alliance = 1;


    @Override
    public void init() {


        localizer = new PinpointLocalizer(
                hardwareMap, Constants.localizerConfig);


        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class, "pinpoint");


        dash = FtcDashboard.getInstance();


        myTe = new MultipleTelemetry(
                dash.getTelemetry(), telemetry);


        intake = new Intake(hardwareMap);
        drivebase = new Drivebase(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);


    }


    @Override
    public void loop() {


        // Alliance selection
        if (gamepad1.xWasPressed()) {
            alliance = alliance == 1 ? 0 : 1;
        }


        boolean aimRequested =
                gamepad1.right_trigger > 0.3 ||
                        gamepad2.right_trigger > 0.3;


        boolean reverseRequested =
                gamepad1.left_trigger > 0.3 ||
                        gamepad2.left_trigger > 0.3;


        // Normal driving is disabled while aiming
        if (aimRequested && !reverseRequested) {


            drivebase.stopRobot();


        } else {


            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;


            drivebase.drive(drive, strafe, turn);
        }


        // Run shooting and aiming logic
        shooting_machine();


        // Update localization
        localizer.update();
        pinpoint.update();


        double heading = pinpoint.getPosition()
                .getHeading(AngleUnit.DEGREES);


        // Telemetry
        myTe.addData("Alliance",
                alliance == 1 ? "RED" : "BLUE");


        myTe.addData("Shooter Velocity",
                shooter.getVelocityP());


        myTe.addData("X", localizer.pose().x());
        myTe.addData("Y", localizer.pose().y());
        myTe.addData("Heading", heading);


        myTe.addData("AprilTag Detected",
                drivebase.hasTarget());


        myTe.addData("AprilTag TX",
                drivebase.getTargetTX());


        myTe.addData("AprilTag Aligned",
                drivebase.isAligned());


        myTe.update();
    }


    public void shooting_machine() {


        boolean reverseRequested =
                gamepad1.left_trigger > 0.3 ||
                        gamepad2.left_trigger > 0.3;


        boolean aimRequested =
                gamepad1.right_trigger > 0.3 ||
                        gamepad2.right_trigger > 0.3;


        // Reverse intake / shooter
        if (reverseRequested) {


            drivebase.stopRobot();


            shooter.setPower(-1.0);
            intake.setPower(1.0);
            transfer.setPower(1.0);


            return;
        }


        // Normal shooter velocity control
        if (shooter.getVelocityP() <
                velocityPTarget * 0.98) {


            shooter.setPower(1.0);


        } else {


            shooter.setVelocityP(velocityPTarget);
        }


        // AprilTag aiming
        if (aimRequested) {


            if (shooter.getVelocityP() >=
                    velocityPTarget * 0.98) {


                boolean aligned =
                        drivebase.turnToAprilTag(alliance);


                if (aligned) {


                    transfer.setPower(-1.0);


                } else {


                    transfer.setPower(0);
                }


            } else {


                drivebase.stopRobot();
                transfer.setPower(0);
            }


            return;
        }


        // Normal intake operation
        if (gamepad1.right_bumper ||
                gamepad2.right_bumper) {


            intake.setPower(-1.0);
            transfer.setPower(0);


        } else {


            intake.setPower(0);
            transfer.setPower(0);
        }
    }
}