^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package reference_system
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

v1.0.0
-----------
* Add first changelog
* Bump version of reference_system packages to 1.0.0
* Update reference_system docs, logic in various places
* Migrate benchmark scripts to python
* [91] skip node graph in reference_system tests for now
* [91] add unit and integration tests for the reference system, fix some bugs found by tests
* Added callback group per subscription in Intersection node.
* patch version bump to include mutex for prints
* Guard cout with mutex
* initial release for each package
* Add missing const keywords
* add std trace type, generate summary report
* Fix latency calculation error, adjust table header
* Uncrustify code
* Add advanced dropped sample statistics for each node
* Add dropped samples statistics
* Add sequence number, behavior planner period, timestamp relates to callback startup
* Add more advanced statistics for latency
* Lint and uncrustify
* Add intersection node docu
* Add intersection node
* fix cpp header guards, timing bug
* Uncrustify and adjust number cruncher
* Cyclic node has number crunching
  * Cyclic node with cyclic trigger implemented
  * Rename Reactor node into Cyclic node
  * Remove time diff check from fusion node
  * Number cruncher uses upper limit instead of timeout, add number cruncher benchmark to help find the best limits for a system
* Add Fusion node max input timediff and fail when it is exceeded
* rename nodes, update docs, add platform test
* clean up cmake, reorg READMEs
* seperate out reference_system vs autoware specifics
* fix uncrustify errors
* Add more fine grained timing configuration
  * fix README links, add requirements doc, linter cleanups
* fix lint errors
* Apply suggestions from code review
* Add benchmark comments and add custom timings in system builder
* Add benchmark timing
* Add benchmark mode
* Uncomment code for single threaded executors
  * update README, clean up system naming
* reorg, lint and clean up
* Move processing times into config
* Remove warnings
* Write number crunch result at first index
* Rename SampleType to SampleTypePointer to make it clear that we expect a pointer
* Config is stored in struct to allow multiple configurations in parallel
* Remove pub/sub port alias
* Rename reference_system_autoware into reference_system
* Contributors: Christian, Christian Eltzschig, Christophe Bedard, Evan Flynn, Lander Usategui, Ralph Lange
