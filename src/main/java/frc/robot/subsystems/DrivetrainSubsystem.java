package frc.robot.subsystems;

import java.util.ArrayList;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;

import com.swervedrivespecialties.swervelib.MechanicalConfiguration;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {

  public static final MechanicalConfiguration MODULE_CONFIG = SdsModuleConfigurations.MK4I_L3;
        
  public static final double MAX_VOLTAGE = 11.0;

  private Field2d field = new Field2d();

  private ArrayList<SwerveModule> modules = new ArrayList<SwerveModule>();

  private SwerveDriveOdometry odometry;
  AHRS ahrs;

  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          MODULE_CONFIG.getDriveReduction() *
          MODULE_CONFIG.getWheelDiameter() * Math.PI;

  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  public final SwerveModule m_frontLeftModule;
  public final SwerveModule m_frontRightModule;
  public final SwerveModule m_backLeftModule;
  public final SwerveModule m_backRightModule;
  public SwerveModuleState[] states;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public DrivetrainSubsystem() {

    SmartDashboard.putData("field", field);

    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            pathPlannerConfig,
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      ahrs = new AHRS(SPI.Port.kMXP); 
    } catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    m_frontLeftModule = new MkSwerveModuleBuilder()
            .withLayout(tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0))
            .withGearRatio(MK4I_L2)
            .withDriveMotor(MotorType.FALCON, FRONT_LEFT_MODULE_DRIVE_MOTOR, "rio")
            .withSteerMotor(MotorType.FALCON, FRONT_LEFT_MODULE_STEER_MOTOR, "rio")
            .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER, "rio")
            .withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET)
            .build();

        modules.add(m_frontLeftModule);

    // We will do the same for the other modules
     m_frontRightModule = new MkSwerveModuleBuilder()
            .withLayout(tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0))
            .withGearRatio(MK4I_L2)
            .withDriveMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_DRIVE_MOTOR, "rio")
            .withSteerMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_STEER_MOTOR, "rio")
            .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER, "rio")
            .withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET)
            .build();
        modules.add(m_frontRightModule);

    m_backLeftModule = new MkSwerveModuleBuilder()
            .withLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0))
            .withGearRatio(MK4I_L2)
            .withDriveMotor(MotorType.FALCON, BACK_LEFT_MODULE_DRIVE_MOTOR, "rio")
            .withSteerMotor(MotorType.FALCON, BACK_LEFT_MODULE_STEER_MOTOR, "rio")
            .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER, "rio")
            .withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET)
            .build();
        modules.add(m_backLeftModule);

    m_backRightModule = new MkSwerveModuleBuilder()
            .withLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0))
            .withGearRatio(MK4I_L2)
            .withDriveMotor(MotorType.FALCON, BACK_RIGHT_MODULE_DRIVE_MOTOR, "rio")
            .withSteerMotor(MotorType.FALCON, BACK_RIGHT_MODULE_STEER_MOTOR, "rio")
            .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER, "rio")
            .withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET)
            .build();
        modules.add(m_backRightModule);

            

   odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation(), getPositions());
  }

  public void zeroGyroscope() {
    ahrs.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {
    return ahrs.getRotation2d();
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.size()];
    for (int i = 0; i < modules.size(); i++) {
      positions[i] = modules.get(i).getPosition();
    }
    return positions;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(getGyroscopeRotation(), getPositions(), pose);
  }

  public ChassisSpeeds getSpeeds() {
    return m_chassisSpeeds;
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }



  @Override
  public void periodic() {
    field.setRobotPose(getPose());

    states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

    odometry.update(getGyroscopeRotation(), getPositions());

    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
    
    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
  }
}