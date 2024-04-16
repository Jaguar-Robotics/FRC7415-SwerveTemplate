# Team 7415 Swerve Template

This is a basic swerve template that runs MK4i modules using democat's swervelib (based on SDS' original swerve lib). Includes pathplanner, functioning odometry, and more. Very easily cusomizable for use with any SDS module/gyro but currently uses navX.

This is fully based on the SDS' original swerve template. We give them full credit.

If you need any help, please reach out to `@jacktajkef` on Chief Delphi, or email `robotics@dths.org`. We'd be happy to help!

## Required Libraries

1. Democat's Swerve Lib
2. PathplannerLib
3. NavX

## Default Control Setup

By default the robot is setup to be controlled by two joysticks. However any xbox controller should work.

The left stick is setup to control the translational movement of the robot using field-oriented control.

The right stick is setup to control the rotational movement of the robot. Right on the stick should make the robot
rotate clockwise while left should make the robot rotate counter-clockwise.

The first button on the left joystick is setup to re-zero the robot's gyroscope. By default, the direction the robot is
facing when turned on is the forwards direction but this can be changed by re-zeroing the gyroscope.

## Configure For Your Robot

1. In the `Constants` class:
    1. Set the `TRACKWIDTH` and `WHEELBASE` to your robot's trackwidth and wheelbase.
    2. Set all of the `*_ANGLE_OFFSET` constants to `-Math.toRadians(0.0)`.
2. Deploy the code to your robot.
    > NOTE: The robot isn't drivable quite yet, we still have to setup the module offsets
3. Turn the robot on its side and align all the wheels so they are facing in the forwards direction.
    > NOTE: The wheels will be pointed forwards (not backwards) when modules are turned so the large bevel gears are towards the right side of the robot. When aligning the wheels they must be as straight as possible. It is recommended to use a long strait edge such as a piece of 2x1 in order to make the wheels straight.
4. Record the angles of each module using the angle put onto Shuffleboard. The values are named
    `Front Left Module Angle`, `Front Right Module Angle`, etc.
5. Set the values of the `*_ANGLE_OFFSET` to `-Math.toRadians(<the angle you recorded>)`
    > NOTE: All angles must be in degrees.
6. Re-deploy and try to drive the robot forwards. All the wheels should stay parallel to each other. If not go back to
    step 3.
7. Make sure all the wheels are spinning in the correct direction. If not, add 180 degrees to the offset of each wheel
    that is spinning in the incorrect direction. i.e `-Math.toRadians(<angle> + 180.0)`.
8. Tune autonomous PID in the `Constants` class.