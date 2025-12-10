// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmRotation;
import frc.robot.subsystems.ArmExtension.ExtensionSetpoint;
import frc.robot.subsystems.ArmRotation.RotationSetpoint;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.constants.SwerveConstants.OIConstants;

public class RobotContainer {
    private final CommandXboxController controller = new CommandXboxController(Constants.xboxControllerPort);
    private final Joystick buttonBoard = new Joystick(Constants.buttonBoardPort);

    // Buttonboard mappings
    private final JoystickButton restButton = new JoystickButton(buttonBoard, 5);
    private final JoystickButton l1Button = new JoystickButton(buttonBoard, 6);
    private final JoystickButton l2Button = new JoystickButton(buttonBoard, 1);
    private final JoystickButton l3Button = new JoystickButton(buttonBoard, 2);
    private final JoystickButton l4Button = new JoystickButton(buttonBoard, 3);

    private final SwerveDrive swerveDrive = new SwerveDrive();

    private final ArmExtension armExtension = new ArmExtension();
    private final ArmRotation armRotation = new ArmRotation();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        swerveDrive.setDefaultCommand(
            swerveDrive.run(
                () -> swerveDrive.drive(
                    -MathUtil.applyDeadband(controller.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(controller.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(controller.getRightX(), OIConstants.kDriveDeadband),
                    true
                )
            )
        );

        controller.leftTrigger().onTrue(
            armExtension.runMotor(controller.getLeftTriggerAxis() * -Constants.maxExtenderOutput)
        );

        controller.rightTrigger().onTrue(
            armExtension.runMotor(controller.getRightTriggerAxis() * Constants.maxExtenderOutput)
        );

        controller.leftBumper().onTrue(
            armRotation.runMotor(-Constants.maxRotatorOutput)
        );

        controller.rightBumper().onTrue(
            armRotation.runMotor(Constants.maxRotatorOutput)
        );

        // Button board bindings
        restButton.onTrue(
            changeState(
                ExtensionSetpoint.Rest,
                RotationSetpoint.Rest
            )
        );

        l1Button.onTrue(
            changeState(
                ExtensionSetpoint.L1,
                RotationSetpoint.L1
            )
        );

        l2Button.onTrue(
            changeState(
                ExtensionSetpoint.L2,
                RotationSetpoint.L2
            )
        );

        l3Button.onTrue(
            changeState(
                ExtensionSetpoint.L3,
                RotationSetpoint.L3
            )
        );

        l4Button.onTrue(
            changeState(
                ExtensionSetpoint.L4,
                RotationSetpoint.L4
            )
        );
    }

    public Command getAutonomousCommand() {
        return none();
    }

    private Command changeState(ExtensionSetpoint extensionSetpoint, RotationSetpoint rotationSetpoint) {
        return either(
            Commands.print("You're already there."), 
            sequence(
                armExtension.extendToSetpoint(ExtensionSetpoint.Rest),
                armRotation.rotateToSetpoint(rotationSetpoint),
                armExtension.extendToSetpoint(extensionSetpoint)
            ), 
            () -> extensionSetpoint == armExtension.extensionSetpoint && rotationSetpoint == armRotation.rotationSetpoint
        );
    }
}
