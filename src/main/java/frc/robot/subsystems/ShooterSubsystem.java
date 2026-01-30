package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LimelightTAMatrix;
import frc.robot.util.ShooterDistanceMatrix;
import frc.robot.util.drivers.LimelightHelpers;
import frc.robot.util.math.MathUtils;

/** Shooter subsystem for controlling the flywheel(s) */

// TODO: Tune limelightcalc

public class ShooterSubsystem extends SubsystemBase {
    public static ShooterSubsystem Instance;

    public double 
    p = 0.1,
    i = 0.01,
    d = 0;
    
    public PIDController pidController;
    /**
     * If true, override drive control with april tag position
     */
    public boolean overrideDrive = false;
    static boolean trigger = false;

    SparkMax sparkMaxA, sparkMaxB;
    AbsoluteEncoder encoderA, encoderB;

    public ShooterSubsystem() {
        Instance = this;

        Log("Shooter subsystem loading...");
        Log("P: " + p + ", I: " + i + ", D: " + d);
        /*pidController = new PIDController(p, i, d);
        pidController.setSetpoint(1);*/

        sparkMaxA = new SparkMax(31, SparkMax.MotorType.kBrushless);
        sparkMaxB = new SparkMax(32, SparkMax.MotorType.kBrushless);

        encoderA = sparkMaxA.getAbsoluteEncoder();
        encoderB = sparkMaxB.getAbsoluteEncoder();

        pidController = new PIDController(p, i, d);
        pidController.setSetpoint(0);
        //pidController.setTolerance(0.05, 0.05);

        LimelightHelpers.setPipelineIndex("limelight", Constants.LIMELIGHT_PIPELINE_ID);
    }

    /**
     * April tag position at hub (if seen)
     * @return limelight horizontal offset to april tag at hub, clamped between -0.8 and 0.8
     */
    public DoubleSupplier aprilTagPos = () -> {
        if (!LimelightHelpers.getTV("limelight") || Constants.SAD_LIMELIGHT_MODE) return 0;

        if (LimelightHelpers.getTID() == 10 || LimelightHelpers.getTID() == 25) {
            Pose3d pose = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
            return pose.getX(); // i think x???
        }

        return 0;
    };

    /**
     * Calculates flywheel speed based on limelight data. If no target, returns 0.85
     * @return flywheel speed (0.05 to 1)
     */
    public double limelightCalculator() {
        if (!LimelightHelpers.getTV("limelight") || Constants.SAD_LIMELIGHT_MODE) return 0.7; // set flywheel speed regardless of vision

        if (LimelightHelpers.getTID() == 10 || LimelightHelpers.getTID() == 26) {
            double pose = LimelightHelpers.getTA("limelight");
            return ShooterDistanceMatrix.get(LimelightTAMatrix.get(pose));
        }


        return 0.7;
    }
    /**
     * Toggle override for drive control
     */
    public Command ToggleOverride() {
        return runOnce(() -> {
            overrideDrive = !overrideDrive;
        });
    }

    public Command triggerThing() {return runOnce(() -> {trigger = !trigger; } ); }

    DoubleSupplier getSetpoint = () -> {
        if (trigger)
            return limelightCalculator();
        else 
            return -0.2;
    };

    /**
     * Command to shoot balls
     */
    public Command Shoot() {
        return run(() -> {
            double beforeClamp = pidController.calculate(encoderA.getVelocity() / Constants.MAX_NEO_VORTEX_SPEED, getSetpoint.getAsDouble()) * 10;
            //var setpoint = getSetpoint.getAsDouble();
            //if (setpoint > 0)
            //    System.out.println("ts: " + setpoint);

            double speed = MathUtils.clamp( beforeClamp, 0, 0.7);

            sparkMaxA.set(speed);
            sparkMaxB.set(speed); // facing same direction (as of 2026-01-24)
        });
    }

    /**
     * Debug command to update PID values
     * @param kP P
     * @param kI I
     * @param kD D
     */
    public Command UpdatePID(double kP, double kI, double kD) {
        return runOnce(() -> {
            pidController.setPID(kP, kI, kD);
            p = kP;
            i = kI;
            d = kD;
        });
    }

    /**
     * Debug command to update PID values
     */
    public Command UpdatePID() {
        return runOnce(() -> {
            pidController.setPID(p, i, d);
        });
    }


    /**
     * Log to console only in test mode
     * @param objects objects to log
     */
    public void Log(Object objects) {
        if (DriverStation.isTestEnabled()) {
            System.out.println(objects);
        }
    }
}
