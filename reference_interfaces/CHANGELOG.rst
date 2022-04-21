^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package reference_interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

v1.0.0
-----------
* Add first changelog
* Bump version of reference_system packages to 1.0.0
* Add test_depend on ament_lint\_{auto,common} for reference_interfaces
* patch version bump to include mutex for prints
* initial release for each package
* Add sequence number, behavior planner period, timestamp relates to callback startup
* Number cruncher uses upper limit instead of timeout, add number cruncher benchmark to help find the best limits for a system
* remove cpu and msg tests for now
* switch target name to wait for idl fix
* add warning and fix for generated idl file bug
* add idl bug work-around as cmake post-build cmd
* seperate out reference_system vs autoware specifics
* Apply suggestions from code review
* Sample tracing implemented
* remove header from msg
* move msgs to their own package, use ament auto
* Contributors: Christian, Christian Eltzschig, Christophe Bedard, Evan Flynn, Lander Usategui
