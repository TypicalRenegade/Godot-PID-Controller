Typical Renegade 2025
TypicalRenegade@proton.me

Ported from https://gist.github.com/mattogodoy/910ef7612950161f4a9871c09b62fec7. Originally in GDScript.

Complete working PID controller module for the Godot Game Engine version 4.0 or higher.

This module does include docs that will appear in Godot's internal editor documentation.

USAGE:
    When compiled in the Godot this module adds a node called "PID". It can be used to controller rigidbody(2D or 3D) movement. PID nodes have two methods that can be called: update() and update_angle(). update() can be called when you want to compare linear values, while update_angle() can be used to compare angles.

    An example of its use is: if you want a rigidbody to retain a specific speed add a PID node as a child of the rigidbody. Then add a script to the rigidbody that takes the rigidbody's current speed, passes that value to the PID controller and the speed to want the rigidbody to attain. E.g. float force = PID_NODE.update(<current_speed>, <target_speed>). The output would be stored in force. Use that number to apply a central force to the rigidbody.

    A PID controller automaticly takes into account all forces being applied to the rigidbody. So the only information to need to have is the current state and the target state. PID node can also be used to apply torque to a rigidbody to control the rotation of a rigidbody or keep the rigidbody turned in a specific direction (e.g. keep a motorcycle rightside up).

    One note: a PID node can only control one axis of movement at a time. So a potential setup to control ship rotation is:
        Three PID nodes (one for each axis): gyro_x, gyro_y, and gyro_z.
        Current angles: angle_x, angle_y, and angle_z.
        Target for all angles is 0.0.
        Code put together: {
        # Roll
        var a_x = gyro_x.update(angle_x, 0.0, delta)
		# Angles for Normal Movement
        var a_y = gyro_y.update_angle(angle_y, 0.0, delta)
        var a_z = gyro_z.update_angle(angle_z, 0.0, delta)
        }

INSTALLATION:
    Download Godot-PID-Controller. Rename the file PID.
    Download Godot source code and place this module under the modules dirtory in the godot source code. Then compile Godot using the instructions from the Godot Docs.
