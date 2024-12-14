package org.firstinspires.ftc.teamcode.TriSwerve;

import com.amarcolini.joos.command.AbstractComponent;
import com.amarcolini.joos.command.Command;
import com.amarcolini.joos.control.DCMotorFeedforward;
import com.amarcolini.joos.control.Feedforward;
import com.amarcolini.joos.control.PIDCoefficients;
import com.amarcolini.joos.control.PIDController;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.hardware.Motor;
import com.amarcolini.joos.util.MathUtil;
import com.amarcolini.joos.util.NanoClock;

@JoosConfig
public class ArmComponent extends AbstractComponent {
    private final Motor slide, pivot;
    private final PIDController pivotController, slideController;
    public static PIDCoefficients pivotCoeffs, slideCoeffs;
    public static double kP, kI, kD, pivotAngleCoeff;
    private double pivotTarget, slideTarget;
    private final double pivotMaxDegs, slideMaxRots;

    private double lastTime, dt;

    public ArmComponent(Motor slide, Motor pivot,
                        PIDCoefficients pivotCoeffs, double pivotAngleCoeff, double pivotMaxDegs,
                        PIDCoefficients slideCoeffs, double slideMaxRots) {
        // Set motors & their respective components
        this.slide = slide;
        this.pivot = pivot;
        subcomponents.add(this.slide);
        subcomponents.add(this.pivot);

        // Set linear slide and pivot mechanism settings
        this.slide.setRunMode(Motor.RunMode.RUN_USING_ENCODER);
        this.pivot.setRunMode(Motor.RunMode.RUN_USING_ENCODER);
        this.slide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //this.pivot.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Set initial target variables
        pivotTarget = pivot.getCurrentPosition() / pivot.TPR;
        slideTarget = slide.getCurrentPosition() / pivot.TPR;
        this.pivotMaxDegs = pivotMaxDegs;
        this.slideMaxRots = slideMaxRots;

        // Create PIDF controllers
        ArmComponent.kP = pivotCoeffs.kP;
        ArmComponent.kI = pivotCoeffs.kI;
        ArmComponent.kD = pivotCoeffs.kD;
        ArmComponent.slideCoeffs = slideCoeffs;
        ArmComponent.pivotCoeffs = pivotCoeffs;
        pivotController = new PIDController(pivotCoeffs);
        slideController = new PIDController(slideCoeffs);
        pivotController.setOutputBounds(-1, 1);
        slideController.setOutputBounds(-1, 1);

        // Time handling
        dt = 0.0;
        lastTime = 0.0;
    }

    // Update super (which updates subcomponents)
    @Override
    public void update() {
        super.update();

        // Update pivot controller for FTC dashboard
        pivotController.setPid(new PIDCoefficients(kP, kI, kD));

        // Calculate PIDF values
        double pivotPos = pivot.getCurrentPosition() / pivot.TPR;
        double slidePos = slide.getCurrentPosition() / slide.TPR;
        pivotController.setTarget(pivotTarget);
        slideController.setTarget(slideTarget);
        double pivotPIDF = pivotController.update(pivotPos);
        double slidePIDF = slideController.update(slidePos);
        pivotPIDF += pivotAngleCoeff * Math.abs(Math.cos(pivotPos * 2 * Math.PI));

        // Set motor power
        pivot.setPower(pivotPIDF);
        slide.setPower(slidePIDF);

        // Telemetry
        telem.addData("PIVOT TARGET", pivotTarget);
        telem.addData("PIVOT CURRENT", pivot.getCurrentPosition() / pivot.TPR);
        telem.addData("PIVOT PIDF", pivotPIDF);
        telem.addData("SLIDE TARGET", slideTarget);
        telem.addData("SLIDE CURRENT", slide.getCurrentPosition() / slide.TPR);
        telem.addData("SLIDE PIDF", slidePIDF);

        // Time handling
        double currentTime = NanoClock.getSystem().seconds();
        dt = currentTime - lastTime;
        lastTime = currentTime;
    }

    // Set motor powers
    public void setPivotTarget(double degrees) {
        pivotTarget = MathUtil.clamp(degrees, 0.0, pivotMaxDegs) / 360;
    }
    public void setSlideTarget(double extension) {
        slideTarget = MathUtil.clamp(extension, 0.0, 1.0) * slideMaxRots;
    }

    // Move motors
    public void movePivotTarget(double degreesPerSecond) {
        setPivotTarget(pivotTarget + degreesPerSecond * dt);
    }
    public void moveSlideTarget(double extensionPerSecond) {
        setSlideTarget(slideTarget + extensionPerSecond * dt);
    }
}