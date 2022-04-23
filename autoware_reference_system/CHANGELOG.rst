^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_reference_system
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

v1.0.0
-----------
* Add first changelog
* Bump version of reference_system packages to 1.0.0
* Skip callback group exe if distro is Foxy
* Update reference_system docs, logic in various places
* Migrate benchmark scripts to python
* clean up reporting code, adjust title and label sizes for figures in reports
* [91] add unit and integration tests for the reference system, fix some bugs found by tests
* Added note on super user privileges.
* Adding autoware_default_prioritized and autoware_default_cbg only to test set if super user rights available.
  Signed-off-by: Ralph Lange <ralph.lange@de.bosch.com>
* Fixed uncrustify finding.
  Signed-off-by: Ralph Lange <ralph.lange@de.bosch.com>
* Do not exit but print warning if thread prioritization fails.
  Signed-off-by: Ralph Lange <ralph.lange@de.bosch.com>
* add skip_tracing cmake arg to readme
* update memory individual report text sizes as well
* increase label sizes for figures
* Under Foxy, exclude executable using callback-group interface of Executor.
  Signed-off-by: Ralph Lange <ralph.lange@de.bosch.com>
* Added executables for prioritized and callback-group-level Executor.
  Signed-off-by: Ralph Lange <ralph.lange@de.bosch.com>
* default to not run benchmark tests
* Make cpu benchmark timings consistent
* switch to use cmake options
* patch version bump to include mutex for prints
* remove extra line
* fix flake8 errors
* return none not no
* handle case where log file line is incomplete
* initial release for each package
* sort axis labels along with data for latency plots
* only run tests for 5s by default
* update dependency list, add warnings to test section
* update node graph
* clean up reports
* add behavior planner jitter
* use candlesticks to show min, max, and std dev
* add std trace type, generate summary report
* fix dropped message count for now
* apply feedback from pr
* fix flake8 errors
* create node graph from list of tuples
* fix flake8 errors
* rebase, refactor report gen, fix dropped msg count
* clean up report generation code
* add prototype latency figure to report
* fix flake8 errors
* clean up report, add expected_count to hovertool
* begin to add dropped message report
* Add advanced dropped sample statistics for each node
* Add dropped samples statistics
* Add sequence number, behavior planner period, timestamp relates to callback startup
* update node graph and svg
* Lint and uncrustify
  * Add intersection node docu
* Add intersection node
  * add all_rmw flag to readme
* move cmake functions to separate files, use default rmw
* fix small grammer mistakes in readme
* adjust timings for more typcial run times
* switch each executor to use default timing config
* Reduce list of enabled tracepoints down to minimum
* add default timeout variable to cmake
* add goals/kpis to readme, rearrange testing sections
* add cmake arg to skip lttng tests
* add title to summary reports, fix callback report gen
* fix cpp header guards, timing bug
* add mem and cpu usage summary report
* add cpu/mem usage reports, add hovertool to plots
* use psrecord for mem and cpu usage
* add onto memory_usage report generation
* clean up memory_usage report generation
* fix env var setting for userspace traces, add memory usage events
* fix ROS_HOME usage in cmake
* change name of generated reports to match parent directory
* clean up cmake variable names
* generate callback_duration report
* Uncrustify and adjust number cruncher
* Cyclic node with cyclic trigger implemented
  * Rename Reactor node into Cyclic node
  * Remove time diff check from fusion node
  * Number cruncher uses upper limit instead of timeout, add number cruncher benchmark to help find the best limits for a system
  fix generating trace files
* fix remaing lint errors
* fix flake8 errors
* use same timing config for all executors
* fix timeout for generating traces
* fix requirements test, update readme
* add report generation from trace files
* add fusion max input time diff to requirements
* Add Fusion node max input timediff and fail when it is exceeded
* add generate_tracing test
* update README, regenerate svg, add graphviz section to README
* fix lint errors, rename tests
* list exes in cmake and loop over them
* add initial pub/sub test
* rename nodes, update docs, add platform tes
* fix readme links after reorg
* remove cpu and msg tests for now
  seperate out reference_system vs autoware specifics
* clean up cmake, reorg READMEs
* seperate out reference_system vs autoware specifics
* Contributors: Christian, Christian Eltzschig, Christophe Bedard, Evan Flynn, Lander Usategui, Ralph Lange
