package com.spartronics4915.frc2023.commands;

import com.spartronics4915.frc2023.PhotonCameraWrapper;
import com.spartronics4915.frc2023.subsystems.Swerve;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import static com.spartronics4915.frc2023.Constants.Swerve.*;

import java.lang.reflect.Field;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import static com.spartronics4915.frc2023.Constants.OI.*;

public class SwerveCommands {
    private final XboxController mController;

    private final Swerve mSwerve;
    private boolean mIsSlowMode = false;

    public SwerveCommands(XboxController controller, Swerve swerve) {
        mController = controller;
        mSwerve = swerve;
    }

    public class ResetCommand extends InstantCommand {
        public ResetCommand() {
            addRequirements(mSwerve);
        }
		
		@Override
		public void initialize() {
            System.out.println("****Reset called****");
			super.initialize();
			mSwerve.resetToAbsolute();
            mSwerve.resetYaw();
			mSwerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(0))); // for odometry testing
            mSwerve.stop();
			mSwerve.alignModules();
		}
    }

    public class SetFieldRelative extends InstantCommand {
		private boolean mFieldRelative;

        public SetFieldRelative(boolean fieldRelative) {
			mFieldRelative = fieldRelative;
        }

		@Override
		public void initialize() {
			super.initialize();
			
            mSwerve.setFieldRelative(mFieldRelative);
		}
    }

    public class ToggleFieldRelative extends InstantCommand {
        public ToggleFieldRelative() {

		}

		@Override
		public void initialize() {
			super.initialize();
			mSwerve.toggleFieldRelative();
		}
    }

    public class ResetYaw extends InstantCommand {
        public ResetYaw() {

		}
		@Override
		public void initialize() {
			super.initialize();
			mSwerve.resetYaw();
		}
    }

    public class ResetOdometry extends InstantCommand {
        public ResetOdometry() {

		}
		
		@Override
		public void initialize() {
			super.initialize();
            mSwerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
		}
    }


    public class TeleopCommand extends CommandBase {
        public TeleopCommand() {
            addRequirements(mSwerve);
        }

        @Override
        public void initialize() {}

        @Override
        public void execute() {
            // System.out.println("teleop");
            double x1 = mController.getLeftX();
            double y1 = mController.getLeftY();
            double x2 = mController.getRightX();
            
            // System.out.println("x1,y1,x2:" + x1 + "," + y1 + "," + x2);
            x1 = applyTransformations(x1);
            y1 = applyTransformations(y1);
            x2 = applyTransformations(x2);
            // System.out.println("x1,y1,x2:" + x1 + "," + y1 + "," + x2);

            Translation2d translation = new Translation2d(-y1, -x1).times(kMaxSpeed);
            double rotation = -x2 * kMaxAngularSpeed;
            // System.out.println("translation,rotation" + translation + "," + rotation);

            if (Math.abs(mController.getRawAxis(kSlowModeAxis)) <= kTriggerDeadband) { // <= for slow mode default
                translation = translation.times(kSlowModeSpeedMultiplier);
                rotation *= kSlowModeAngularSpeedMultiplier;
            }
            
            // System.out.println("translation,rotation" + translation + "," + rotation);
            mSwerve.drive(translation, rotation, true);
        }

        @Override
        public void end(boolean interrupted) {}

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    public class TestInitCommand extends CommandBase {
        public TestInitCommand() {
            addRequirements(mSwerve);
        }

        @Override
        public void initialize() {}

        @Override
        public void execute() {}

        @Override
        public void end(boolean interrupted) {}

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public class TestCommand extends CommandBase {
        public TestCommand() {
            addRequirements(mSwerve);
        }

        @Override
        public void initialize() {}

        @Override
        public void execute() {}

        @Override
        public void end(boolean interrupted) {}

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    private double applyTransformations(double c) {
        return applyResponseCurve(MathUtil.applyDeadband(c, kStickDeadband));
    }

    private double applyResponseCurve(double c) {
        return Math.signum(c) * Math.pow(Math.abs(c), kResponseCurveExponent);
    }

    public class RotateDegrees extends CommandBase {

        private double mDegreeRotate;
        public RotateDegrees(double degrees) {
            mDegreeRotate = degrees;
        }

        @Override
        public void initialize() {
            var yaw = mSwerve.getYaw();

            Rotation2d newYaw = yaw.plus(Rotation2d.fromDegrees(mDegreeRotate));
            var newCommand = new RotateToYaw(newYaw);
            newCommand.schedule();
        }
    }

    public class RotateToTarget extends CommandBase {;
        private final double mYawToleranceDegrees = 10;
        private final double mAngularVelToleranceDegreesSec = 1;
        private final ProfiledPIDController pid;

        public RotateToTarget() {
            pid = new ProfiledPIDController(0.02, 0, 0.01, new TrapezoidProfile.Constraints(
                0.1,
                kMaxAcceleration
            ));
            pid.setTolerance(mYawToleranceDegrees, mAngularVelToleranceDegreesSec);
            pid.enableContinuousInput(0, 360);

            addRequirements(mSwerve);
        }

        @Override
        public void initialize() {
            PhotonPipelineResult result = Swerve.mCameraWrapper.photonCamera.getLatestResult();
            if (result.hasTargets()) {
                // var yaw = mSwerve.getYaw();
                // System.out.println("Current Yaw: " + yaw.getDegrees());
                // double targetYaw = result.getBestTarget().getYaw();
                double targetYaw = result.getBestTarget().getYaw();
                // System.out.println("Tag Yaw: " + targetYaw);
                // Rotation2d newYaw = yaw.minus(Rotation2d.fromDegrees(targetYaw));
                // System.out.println("Goal Yaw: " + newYaw.getDegrees());
                var newCommand = new RotateYaw(Rotation2d.fromDegrees(targetYaw));
                newCommand.schedule();
            }
        }

        // @Override
        // public void execute() {
        //     PhotonPipelineResult result = Swerve.mCameraWrapper.photonCamera.getLatestResult();
        //     if (result.hasTargets()) {
        //         double tagYaw = result.getBestTarget().getYaw();
        //         // double d = pid.calculate(tagYaw, 0);
        //         // mSwerve.drive(
        //         //     new Translation2d(),
        //         //     -d,
        //         //     true
        //         // );
        //         System.out.println(tagYaw);
        //     }
        // }

        @Override
        public boolean isFinished() {
            boolean positionFine = (Math.abs(pid.getPositionError()) < pid.getPositionTolerance());
            boolean velocityFine = (Math.abs(pid.getVelocityError()) < pid.getVelocityTolerance());
            Boolean finished = positionFine && velocityFine;
            if (finished) {
                System.out.println("done");
            }
            return finished;
        }
    }

    public class RotateToYaw extends CommandBase {

        private final double mYawToleranceDegrees = 2;
        private final double mAngularVelToleranceDegreesSec = 1;
        private Rotation2d mDestinationYaw;
        private final ProfiledPIDController pid;

        public RotateToYaw(Rotation2d destinationYaw) {
            pid = new ProfiledPIDController(0.02, 0, 0, 
            new TrapezoidProfile.Constraints(4, 1.0));
            pid.setTolerance(mYawToleranceDegrees, mAngularVelToleranceDegreesSec);
            pid.enableContinuousInput(-180, 180);
            
            addRequirements(mSwerve);
            mDestinationYaw = destinationYaw.times(-1);
        }

        @Override
        public void initialize() {
            //pid.reset(mSwerve.getYaw().getDegrees());
        }
//zack was here
        @Override
        public void execute() {
            double goal = mDestinationYaw.getDegrees();
            double yaw = mSwerve.getYaw().getDegrees();
            double d = pid.calculate(yaw, goal);
            SmartDashboard.putNumber("Yaw", yaw);
            SmartDashboard.putNumber("Goal", goal);
            SmartDashboard.putNumber("Theta", d);
            mSwerve.drive(
                new Translation2d(),
                d,
                true
            );
        }

        @Override
        public boolean isFinished() {
            boolean positionFine = (Math.abs(pid.getPositionError()) < pid.getPositionTolerance());
            // boolean velocityFine = (Math.abs(pid.getVelocityError()) < pid.getVelocityTolerance());
            Boolean finished = positionFine;
            if (finished) {
                System.out.println("done");
            }
            return finished;
        }

    }

    public class RotateYaw extends CommandBase {

        private final double mYawToleranceDegrees = 10;
        private final double mAngularVelToleranceDegreesSec = 1;
        private final double kP = 0.2;
        private Rotation2d mDeltaYaw;
        private final ProfiledPIDController pid;

        public RotateYaw(Rotation2d deltaYaw) {
            pid = new ProfiledPIDController(kP, 0, 0.01, new TrapezoidProfile.Constraints(
                10,
                kMaxAcceleration
            ));
            pid.setTolerance(mYawToleranceDegrees, mAngularVelToleranceDegreesSec);
            pid.enableContinuousInput(0, 360);

            addRequirements(mSwerve);
            mDeltaYaw = deltaYaw;
        }

        @Override
        public void execute() {
            double d = pid.calculate(mDeltaYaw.getDegrees(), 0);
            //System.out.println(d);
            mSwerve.drive(
                new Translation2d(),
                -d,
                true
            );
        }

        @Override
        public boolean isFinished() {
            boolean positionFine = (Math.abs(pid.getPositionError()) < pid.getPositionTolerance());
            boolean velocityFine = (Math.abs(pid.getVelocityError()) < pid.getVelocityTolerance());
            Boolean finished = positionFine && velocityFine;
            if (finished) {
                System.out.println("done");
            }
            return finished;
        }

    }

}