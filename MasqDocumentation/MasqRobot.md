MasqRobot
===================


This Documents all major changes to the MasqRobot Class.

----------


Changes and Commits
-------------
v0.0.3

Commit Tag --> e62e9f529988bc4c83e6015e0530f8f2fc7f027a

> **Changes:**
> - Teleop modes are closed loop.

v0.0.2

Commit Tag --> f36083621aedff8645487abd0540aa0581c65034

> **Changes:**
> - Made a method that returns encoded position.

v0.0.1

Commit Tag --> 7b6334bb93ba4a4719972fba09f4e4bff33f339f -->Added runToPositon()

> **Changes:**
> - Added runToPosition method to the MasqRobot Class
> - Adds a drive method that does not take imu input and correct angle so we have fall beck if anything goes wrong with our our imu, or any other problem that may occur either hardware, or software.

