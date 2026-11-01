'''
Q8botOne in MuJoCo: the model in model/q8botone.xml, and tools around it.

    kinematics  a leg's servo angles <-> foot position, and the chassis
                moving over planted feet - in the q8bot library's conventions
    sim         the model loaded, with servo targets in degrees, feet, the
                chassis's pose and the test stand
    firmware    the robot's firmware redone against the model, and a stand-in
                for the radio link - so the q8bot library drives the sim
    imu         an MPU6050 on the robot: its mounting calibrated, the chassis's
                orientation tracked from it, and a simulated one
    qtview      the model drawn into a Qt widget
    poser       a window for posing and driving the robot
'''
