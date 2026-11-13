package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.subsystems.Intake.blockN;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.gateN;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.gateP;


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



    // Shooter targets (ticks per second)
    public static double velocityNTarget = 1000;
    public static double velocityPTarget = 1250;



    // Shooter recovery threshold
    public static double shooterRecoveryThreshold = 0.98;



    // Shooting gates
    public static double gateNClosed = -0.5;
    public static double gateNOpen = 0.0;


    public static double gatePClosed = 0.0;
    public static double gatePOpen = 1.0;



    // Block N
    public static double blockNClosed = 1.0;
    public static double blockNOpen = -1.0;


    private boolean blockNIsOpen = true;



    // Intake
    public static double intakePower = 1.0;
    public static double triggerThreshold = 0.3;



    // Alliance
    // 0 = Blue, 1 = Red
    private int alliance = 1;





    @Override
    public void init() {


        localizer = new PinpointLocalizer(
                hardwareMap,
                Constants.
                        localizerConfig
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



// Initialize block N
        blockNIsOpen = true;
        updateBlockN();



// Close shooting gates
        closeGates();



// Stop intake
        intake.setPower(0);


        myTe.addLine("TeleOp Initialized");
        myTe.update();
    }





    @Override
    public void loop() {


        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;


        drivebase.drive(drive, strafe, turn);



// Triangle = Close block N
        if (gamepad1.triangleWasPressed()) {


            blockNIsOpen = false;
            updateBlockN();
        }



// Circle = Open block N
        if (gamepad1.circleWasPressed()) {


            blockNIsOpen = true;
            updateBlockN();
        }


        shooting_machine(false, false);


        localizer.update();
        pinpoint.update();


        double heading = pinpoint.getPosition()
                .getHeading(AngleUnit.DEGREES);




        double velocityP = shooter.getVelocityP();
        double velocityN = shooter.getVelocityN();


        boolean pReady =
                velocityP >= velocityPTarget * shooterRecoveryThreshold;


        boolean nReady =
                velocityN >= velocityNTarget * shooterRecoveryThreshold;


        myTe.addData("Shooter P Velocity", velocityP);
        myTe.addData("Shooter N Velocity", velocityN);


        myTe.addData("Target P Velocity", velocityPTarget);
        myTe.addData("Target N Velocity", velocityNTarget);


        myTe.addData("P Ready", pReady);
        myTe.addData("N Ready", nReady);


        myTe.addData("Shooter Ready", pReady && nReady);


        myTe.addData(
                "Block N",
                blockNIsOpen ? "OPEN" :
                        "CLOSED"
        );


        myTe.addData("X", localizer.pose().x());
        myTe.addData("Y", localizer.pose().y());
        myTe.addData("Heading", heading);


        myTe.update();
    }


    public void shooting_machine(
            boolean aimRequested,
            boolean reverseRequested) {


        double velocityP = shooter.getVelocityP();
        double velocityN = shooter.getVelocityN();



// Each motor maintains its own target.
        // No switching between full power and velocity mode.
        shooter.setVelocityP(velocityPTarget);
        shooter.setVelocityN(velocityNTarget);


        boolean pReady =
                velocityP >= velocityPTarget * shooterRecoveryThreshold;


        boolean nReady =
                velocityN >= velocityNTarget * shooterRecoveryThreshold;


        boolean shooterReady = pReady && nReady;


        boolean shootRequested =
                gamepad1.right_trigger > triggerThreshold;


        boolean intakeForward =
                gamepad1.right_bumper;


        boolean intakeReverse =
                gamepad1.left_trigger > triggerThreshold;


        if (shootRequested && shooterReady) {


            gateN.setPosition(gateNOpen);
            gateP.setPosition(gatePOpen);


            intake.setPower(-intakePower);


            return;
        }




        if (intakeReverse) {


            closeGates();


            intake.setPower(intakePower);


            return;
        }




        if (intakeForward) {


            intake.setPower(-intakePower);


        } else {


            intake.setPower(0);
        }




        closeGates();
    }


    private void updateBlockN() {


        if (blockNIsOpen) {


            blockN.setPower(blockNOpen);


        } else {


            blockN.setPower(blockNClosed);
        }
    }


    private void closeGates() {


        gateN.setPosition(gateNClosed);
        gateP.setPosition(gatePClosed);
    }



    @Override
    public void stop() {


        drivebase.stopRobot();


        intake.setPower(0);


        shooter.stop();


        closeGates();
    }
}