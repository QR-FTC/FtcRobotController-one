package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "VisionOpModeModular")
public class VisionOpModeModular extends LinearOpMode {

    // Hardware and Vision Objects
    private DcMotor intakeSlide;
    private OpenCvWebcam camera;
    private ObjectDetectionServo pipeline;
    private PixeltoDistanceMapper mapper;

    // Constants for calculations
    // These can now be easily tuned in one place
    private static final double SPOOL_EFFECTIVE_CIRCUMFERENCE_INCHES = (1.5748 * 44) / 7.0;
    private static final double SLIDE_TICKS_PER_REVOLUTION = -154.1;
    private static final double SLIDE_TICKS_PER_INCH = SLIDE_TICKS_PER_REVOLUTION / SPOOL_EFFECTIVE_CIRCUMFERENCE_INCHES;
    private static final double SLIDE_POWER = 0.8;

    @Override
    public void runOpMode() {
        // --- INITIALIZATION ---
        // Each setup step is now a clear, single-line method call
        initializeHardware();
        initializeDistanceMapper();
        initializeVision();

        telemetry.addLine("✅ Initialization Complete. Waiting for start.");
        telemetry.update();

        waitForStart();

        // --- MAIN LOOP ---
        // The main loop is now clean and easy to read
        while (opModeIsActive()) {
            Point centerPoint = pipeline.getCenter();

            // Get the calculated distance from the mapper
            PixeltoDistanceMapper.DistanceResult inchDistance = getDistanceResult(centerPoint);

            // Perform calculations using the distance result
            int slideTargetPosition = calculateSlideTargetPosition(inchDistance);
            double servoTargetPosition = calculateServoPosition(pipeline.getAngle());

            // Command the hardware
            moveSlideToPosition(slideTargetPosition);

            // Update telemetry
            updateTelemetry(centerPoint, inchDistance, slideTargetPosition, servoTargetPosition);
        }

        // --- CLEANUP ---
        camera.stopStreaming();
    }

    // --- Initialization Methods ---

    /**
     * Initializes all hardware components from the hardware map.
     */
    private void initializeHardware() {
        intakeSlide = hardwareMap.get(DcMotor.class, "intakeSlide");
        intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // blueServo = hardwareMap.get(Servo.class, "blue"); // Uncomment if you add the servo back
    }

    /**
     * Initializes the PixeltoDistanceMapper with hardcoded calibration data.
     */
    private void initializeDistanceMapper() {
        double[][] calibrationData = {
                {1603, 907, 5, -7, 15},
                {750, 887, 5.5, -7, 11.5},
                {879, 363, 25.5, -7.5, 29},
                {1195, 367, 26, 7, 29},
                {1072, 508, 15, -1.5, 19}
        };
        mapper = new PixeltoDistanceMapper(calibrationData);
    }

    /**
     * Sets up the webcam, pipeline, and starts the camera stream.
     */
    private void initializeVision() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new ObjectDetectionServo();
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1920, 1200, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("❌ Camera Error", errorCode);
                telemetry.update();
            }
        });
    }

    // --- Calculation Methods (Easily Testable) ---

    /**
     * Gets the distance result from the mapper, handling null (no object detected) cases.
     * @param centerPoint The center point of the detected object from the vision pipeline.
     * @return The calculated DistanceResult.
     */
    private PixeltoDistanceMapper.DistanceResult getDistanceResult(Point centerPoint) {
        if (centerPoint != null) {
            return mapper.getDistanceFromPixel(centerPoint.x, centerPoint.y);
        } else {
            // Return a default or "zero" state if no object is detected
            return mapper.getDistanceFromPixel(0, 0);
        }
    }

    /**
     * Calculates the target encoder tick position for the intake slide.
     * This is a "pure function" that is very easy to unit test.
     * @param inchDistance The distance data for the detected object.
     * @return The target position in encoder ticks.
     */
    public int calculateSlideTargetPosition(PixeltoDistanceMapper.DistanceResult inchDistance) {
        double forward = inchDistance.forwardDist - 10;
        double lateral = inchDistance.horizOffset;

        // Ensure the value inside sqrt is not negative
        double lateralSquared = lateral * lateral;
        if (lateralSquared > 81) {
            lateralSquared = 81;
        }

        double v = Math.sqrt(81 - lateralSquared) + forward;
        return (int) (SLIDE_TICKS_PER_INCH * v);
    }

    /**
     * Converts the detected angle from the pipeline to a servo position.
     * This is also a pure, easily testable function.
     * @param angle The angle in degrees (0-180) from the vision pipeline.
     * @return The target position for a servo (e.g., 0.0 to 1.0).
     */
    public double calculateServoPosition(double angle) {
        // This is a linear mapping from angle [0, 180] to servo position [0.0989, 0.7661]
        return (((angle) * (0.7661 - 0.0989)) / 180.0) + 0.0989;
    }

    // --- Action & Telemetry Methods ---

    /**
     * Commands the intake slide motor to move to a specified position.
     * @param targetPosition The target position in encoder ticks.
     */
    private void moveSlideToPosition(int targetPosition) {
        intakeSlide.setTargetPosition(targetPosition);
        intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeSlide.setPower(SLIDE_POWER);
    }

    /**
     * Displays all relevant data on the Driver Station.
     */
    private void updateTelemetry(Point centerPoint, PixeltoDistanceMapper.DistanceResult dist, int slideTarget, double servoTarget) {
        telemetry.addLine("--- Vision ---");
        if (centerPoint != null) {
            telemetry.addData("X-Value", "%.2f", centerPoint.x);
            telemetry.addData("Y-Value", "%.2f", centerPoint.y);
        } else {
            telemetry.addLine("Object not detected");
        }
        telemetry.addData("Angle", "%.2f", pipeline.getAngle());

        telemetry.addLine("\n--- Calculations ---");
        telemetry.addData("Forward Dist (in)", "%.2f", dist.forwardDist);
        telemetry.addData("Lateral Dist (in)", "%.2f", dist.horizOffset);
        telemetry.addData("Servo Target", "%.3f", servoTarget);

        telemetry.addLine("\n--- Motor ---");
        telemetry.addData("Slide Target Ticks", slideTarget);
        telemetry.addData("Slide Current Ticks", intakeSlide.getCurrentPosition());

        telemetry.update();
    }
}