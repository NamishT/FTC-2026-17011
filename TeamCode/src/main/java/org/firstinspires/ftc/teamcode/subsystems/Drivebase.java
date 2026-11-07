

package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


//import org.firstinspires.ftc.teamcode.subsystems.limeLightV2;


public class Drivebase {


    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;


    private GoBildaPinpointDriver pinpoint;
//    private limeLightV2 lime;


    // Dashboard tuning variables

//    public static double MAX_TURN_POWER = 0.30;
//
//
//    public static double FILTER_ALPHA = 0.25;
//
//
//    public static double AIM_ENTER_TOLERANCE = 3.0;
//    public static double AIM_EXIT_TOLERANCE = 4.0;
//
//
//    // Aiming information
//    private double filteredTX = 0;
//    private double currentTX = 0;


//    private boolean filterInitialized = false;
//    private boolean targetFound = false;
//    private boolean aligned = false;


//    private int selectedTagID = -1;


    public Drivebase(HardwareMap hardwareMap) {


        frontLeft = hardwareMap.get(
                DcMotor.class, "frontLeft");


        backLeft = hardwareMap.get(
                DcMotor.class, "backLeft");


        frontRight = hardwareMap.get(
                DcMotor.class, "frontRight");


        backRight = hardwareMap.get(
                DcMotor.class, "backRight");


        frontLeft.setDirection(
                DcMotor.Direction.FORWARD);


        backLeft.setDirection(
                DcMotor.Direction.FORWARD);


        frontRight.setDirection(
                DcMotor.Direction.REVERSE);


        backRight.setDirection(
                DcMotor.Direction.REVERSE);


        frontLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);


        frontRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);


        backLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);


        backRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);


        pinpoint  = hardwareMap.get(
                GoBildaPinpointDriver.class, "pinpoint");


//        lime = new limeLightV2(hardwareMap);
    }


    public void drive(
            double drive,
            double strafe,
            double turn) {


        double denominator = Math.max(
                Math.abs(drive) +
                        Math.abs(strafe) +
                        Math.abs(turn),
                1.0
        );


        double frontLeftPower =
                (drive + strafe + turn) / denominator;


        double frontRightPower =
                (drive - strafe - turn) / denominator;


        double backLeftPower =
                (drive - strafe + turn) / denominator;


        double backRightPower =
                (drive + strafe - turn) / denominator;


        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }


//    public boolean turnToAprilTag(int allianceType) {


//        double[][] tags = lime.limeLight(allianceType);


//        double closestDistance = Double.MAX_VALUE;


//        double targetAngle = 0;
//        int newTagID = -1;


//        targetFound = false;


        // Find closest visible alliance tag
//        for (double[] tag : tags) {
//
//
//            if (tag == null || tag.length < 3) {
//                continue;
//            }
//
//
//            if (tag[0] < closestDistance) {
//
//
//                closestDistance = tag[0];


//                targetAngle = tag[1];
//
//
//                newTagID = (int) tag[2];
//
//
//                targetFound = true;
//            }
//        }


        // No AprilTag visible
//        if (!targetFound) {
//
//
//            currentTX = 0;
//            selectedTagID = -1;
//
//
//            filterInitialized = false;
//            aligned = false;
//
//
//            stopRobot();
//
//
//            return false;
//        }
//
//
//        // Reset filter if the selected tag changes
//        if (newTagID != selectedTagID) {
//
//
//            selectedTagID = newTagID;
//
//
//            filteredTX = targetAngle;
//
//
//            filterInitialized = true;
//
//
//            aligned = false;
//        }
//
//
//        // Smooth camera measurements
//        if (!filterInitialized) {
//
//
//            filteredTX = targetAngle;
//            filterInitialized = true;
//
//
//        } else {
//
//
//            filteredTX =
//                    FILTER_ALPHA * targetAngle +
//                            (1.0 - FILTER_ALPHA) * filteredTX;
//        }
//
//
//        currentTX = filteredTX;
//
//
//        // Alignment hysteresis
//        if (aligned) {
//
//
//            if (Math.abs(filteredTX) <= AIM_EXIT_TOLERANCE) {
//
//
//                stopRobot();
//
//
//                return true;
//            }
//
//
//            aligned = false;
//        }
//
//
//        // Enter alignment zone
//        if (Math.abs(filteredTX) <= AIM_ENTER_TOLERANCE) {
//
//
//            aligned = true;
//
//
//            stopRobot();
//
//
//            return true;
//        }
//
//
//        // Proportional controller
//        double turnPower = (filteredTX<0) ? -1: 1;
//
//
//        // Limit turning power
//        turnPower = Math.max(
//                -MAX_TURN_POWER,
//                Math.min(MAX_TURN_POWER, turnPower)
//        );


//        turnRobot(turnPower);


//        return false;
//    }


    public void turnRobot(double power) {


        // Same rotation convention as drive()
        frontLeft.setPower(power);
        backLeft.setPower(power);


        frontRight.setPower(-power);
        backRight.setPower(-power);
    }


    public void stopRobot() {


        frontLeft.setPower(0);
        backLeft.setPower(0);


        frontRight.setPower(0);
        backRight.setPower(0);
    }


    public void resetAim() {


        stopRobot();
//
//
//        filterInitialized = false;
//        targetFound = false;
//        aligned = false;
//
//
//        selectedTagID = -1;
//
//
//        currentTX = 0;
//        filteredTX = 0;
    }


//    public double getTargetTX() {
//        return currentTX;
//    }


//    public boolean hasTarget() {
//        return targetFound;
//    }


//    public boolean isAligned() {
//        return aligned;
//    }


//    public int getSelectedTagID() {
//        return selectedTagID;
//    }


//    public boolean isCameraValid() {
//        return lime.isCameraValid();
//    }
}