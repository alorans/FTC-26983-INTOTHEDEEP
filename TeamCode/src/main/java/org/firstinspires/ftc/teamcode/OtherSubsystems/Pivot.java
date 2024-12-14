package org.firstinspires.ftc.teamcode.OtherSubsystems;

import com.amarcolini.joos.command.Component;
import com.amarcolini.joos.control.DCMotorFeedforward;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.hardware.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleSupplier;
@JoosConfig
public class Pivot implements Component {
    // Objects
    private final HardwareMap hardwareMap;
    private final DcMotor pivotMotor;
    private  double currentPos;
    private double targetPos;
    private DoubleSupplier controlLY;
    private static DCMotorFeedforward feedforward;
    public static double kV, kA, kStatic;



    // Default constructor
    public Pivot(HardwareMap hardware, String pivotMotorId, DoubleSupplier controlLY, double kV, double kA, double kStatic){
        this.hardwareMap = hardware;
        this.pivotMotor = hardwareMap.dcMotor.get(pivotMotorId);

        this.pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        targetPos=this.pivotMotor.getCurrentPosition();

        this.controlLY = controlLY;
        this.kV=kV; this.kA=kA; this.kStatic = kStatic;
        feedforward = new DCMotorFeedforward(kV,kA,kStatic);


    }



    // Always maintain pos
    @Override
    public void update(){

        if (Math.abs(controlLY.getAsDouble()) > 0.05) {

            // Move the motor based on joystick input
            pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            pivotMotor.setPower(-controlLY.getAsDouble());

            // Update the target position for holding
            targetPos = pivotMotor.getCurrentPosition();

        } else {
            pivotMotor.setTargetPosition((int) targetPos);
            // Hold position
            pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            pivotMotor.setPower(1); // Adjust power as needed for holding
        }


    }

    // FTC-dashboard debug functions
    public double getCurrentPos() {
        return feedforward.calculate(controlLY.getAsDouble(),1);
    }
}


