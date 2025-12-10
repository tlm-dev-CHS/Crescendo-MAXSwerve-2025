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

public class ArmExtension extends SubsystemBase {
    private final SparkMax armExtender = new SparkMax(CAN_IDs.armExtender, MotorType.kBrushless);
    private final RelativeEncoder extenderEncoder = armExtender.getEncoder();

    private final SparkMaxConfig extenderConfig = new SparkMaxConfig();  
    private final SparkClosedLoopController extenderCLC = armExtender.getClosedLoopController();

    public ExtensionSetpoint extensionSetpoint;

    public enum ExtensionSetpoint {
        Rest(0),
        
        L1(Setpoints.L1.extenderHeight),
        L2(Setpoints.L2.extenderHeight),
        L3(Setpoints.L3.extenderHeight),
        L4(Setpoints.L4.extenderHeight);

        public double height;

        private ExtensionSetpoint(double height) {
            this.height = height;
        }
    }

    public ArmExtension() {
        extenderConfig
            .idleMode(IdleMode.kBrake)
        .encoder
            .apply(Configs.ArmExtension.encoderConfig);

        extenderConfig.closedLoop
            .apply(Configs.ArmExtension.closedLoopConfig)
            .apply(Configs.ArmExtension.motionConfig);

        armExtender.configure(extenderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getHeight() {
        return extenderEncoder.getPosition();
    }

    public Command runMotor(double speed) {
        return run(
            () -> armExtender.set(speed)
        );
    }

    public Command stop() {
        return run(
            () -> armExtender.stopMotor()
        );
    }

    public Command extendToSetpoint(ExtensionSetpoint extensionSetpoint) {
        return run(
            () -> {
                this.extensionSetpoint = extensionSetpoint;

                extenderCLC.setReference(
                    extensionSetpoint.height, 
                    ControlType.kMAXMotionPositionControl
                );
            }
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Extender Amps", armExtender.getOutputCurrent());

        SmartDashboard.putNumber("Extender Height", getHeight());
        SmartDashboard.putString("Current Extension Setpoint", extensionSetpoint.name());
    }
}
