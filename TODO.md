TODO:
- Update SparkMAX CAN IDs
    - open rev hardware client while plugged in with orange cable
    - change ID to match the code
- debug strange driving behavior
    - try adding joystick deadzone
    - probably solved by updating IDs (above task)
- Configure TalonFX, CANCoder
    - set breakmode
    - set sensor units to pulses


Additional TODO:
- Add buttons to adjust swerve field orientation "trim" if it gets knocked out of alignment



Stretch goals TODO:
- Report back absolute position estimation using AprilTags
- Use (optionally Kalman filters) and odometry history to create more accurate prediction of current position to use as feedback for autonomous driving control for path following

autonomous mode
- drive out of zone
   - based on encoder position absolute position on the field



2/1/2024
- continue debugging unit conversions in SwerveModule
need to verify that a 180 degree turn matches the expected number of pulses