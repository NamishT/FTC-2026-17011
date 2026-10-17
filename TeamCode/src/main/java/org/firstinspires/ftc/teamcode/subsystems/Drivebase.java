
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivebase {

    // Declare motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver pinpoint;
//    private Limelight3A limelight;

    private double offset = 0;
    private double IMUheadingTracker = 0;

    private static final double AIM_KP = 0.02;
    private static final double AIM_MAX_POWER = 0.30;
    private static final double AIM_MIN_POWER = 0.08;
    private static final double AIM_TOLERANCE = 1.0; // degrees

    private double TX = 0;

    // Constructor
    public Drivebase(HardwareMap hardwareMap) {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Motor directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Pinpoint
        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class, "pinpoint"
        );

        // Initialize Limelight
//        limelight = hardwareMap.get(
//                Limelight3A.class, "limelight"
//        );
//
//        limelight.pipelineSwitch(0);
//        limelight.start();
    }

    public void offsetYaw(double offset) {
        this.offset = Math.toRadians(offset);
    }

    // Mecanum drive
    public void drive(double drive, double strafe, double turn) {
        double denominator = Math.max(
                Math.abs(drive) + Math.abs(strafe) + Math.abs(turn),
                1.0
        );

        double frontLeftPower = (drive + strafe + turn) / denominator;
        double backLeftPower = (drive - strafe + turn) / denominator;
        double frontRightPower = (drive - strafe - turn) / denominator;
        double backRightPower = (drive + strafe - turn) / denominator;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    // Aim at a visible AprilTag using Limelight
    public boolean aimAtAprilTag(LLResult result) {
        if (result == null || !result.isValid()) {
            stop();
            return false;
        }

        if (result.getFiducialResults() == null
                || result.getFiducialResults().isEmpty()) {
            stop();
            return false;
        }

        // Get horizontal angle to the first detected AprilTag
        LLResultTypes.FiducialResult tag =
                result.getFiducialResults().get(0);

        TX = tag.getTargetXDegrees();

        // Stop turning when the tag is centered
        if (Math.abs(TX) <= AIM_TOLERANCE) {
            stop();
            return true;
        }

        // Calculate proportional turn power
        double turn = AIM_KP * TX;

        // Limit turning power
        turn = Math.max(-AIM_MAX_POWER,
                Math.min(AIM_MAX_POWER, turn));

        // Ensure enough power to start rotating
        if (Math.abs(turn) < AIM_MIN_POWER) {
            turn = Math.copySign(AIM_MIN_POWER, turn);
        }

        // If it turns away from the target, reverse the sign:
        // drive(0, 0, -turn);
        drive(0, 0, turn);

        return false;
    }

    // Update Pinpoint and return heading in degrees
    public double getBotHeading() {
        pinpoint.update();
        return pinpoint.getPosition().getHeading(
                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES
        );
    }

    public double getTargetX() {
        return TX;
    }

    // Stop all motors
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
