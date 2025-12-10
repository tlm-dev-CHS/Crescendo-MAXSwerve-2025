package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.Setpoints;

public class ArmRotation extends SubsystemBase {
    private final SparkMax armRotator = new SparkMax(CAN_IDs.armRotator, MotorType.kBrushless);
    private final SparkMax armFollower = new SparkMax(CAN_IDs.armFollower, MotorType.kBrushless);

    private final RelativeEncoder armEncoder = armRotator.getEncoder();

    private final SparkMaxConfig rotatorConfig = new SparkMaxConfig();
    private final SparkMaxConfig followerConfig = new SparkMaxConfig();

    private final SparkClosedLoopController rotatorCLC = armRotator.getClosedLoopController();

    public RotationSetpoint rotationSetpoint;

    public enum RotationSetpoint {
        Rest(0),
        
        L1(Setpoints.L1.rotationAngle),
        L2(Setpoints.L2.rotationAngle),
        L3(Setpoints.L3.rotationAngle),
        L4(Setpoints.L4.rotationAngle);

        public double angle;

        private RotationSetpoint(double angle) {
            this.angle = angle;
        }
    }

    public ArmRotation() {
        rotatorConfig
            .idleMode(IdleMode.kBrake)
        .encoder
            .apply(Configs.ArmRotation.encoderConfig);

        rotatorConfig.closedLoop
            .apply(Configs.ArmRotation.closedLoopConfig)
            .apply(Configs.ArmRotation.motionConfig);

        followerConfig
            .idleMode(IdleMode.kBrake)
            .follow(armRotator)
            .inverted(true);

        armRotator.configure(rotatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getAngle() {
        return armEncoder.getPosition();
    }

    public Command runMotor(double speed) {
        return run(
            () -> armRotator.set(speed)
        );
    }

    public Command stop() {
        return run(
            () -> armRotator.stopMotor()
        );
    }

    public Command rotateToSetpoint(RotationSetpoint rotationSetpoint) {
        return run(
            () -> {
                this.rotationSetpoint = rotationSetpoint;

                rotatorCLC.setReference(
                    rotationSetpoint.angle,
                    ControlType.kMAXMotionPositionControl
                );
            }
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Rotator Amps", armRotator.getAppliedOutput());
    
        SmartDashboard.putNumber("Arm Angle", getAngle());
        SmartDashboard.putString("Current Rotation Setpoint", rotationSetpoint.name());
    }
}
