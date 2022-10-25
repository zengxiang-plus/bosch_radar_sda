NUM_JOBS := $(shell nproc)
BUILD_COMMAND=$(MAKE) --jobs $(NUM_JOBS) --no-print-directory

.PHONY: radar_test
radar_test:
	mkdir -p src/bosch_radar/build && cd src/bosch_radar/build && \
    cmake -DCMAKE_BUILD_TYPE="RelWithDebInfo" .. && \
    $(BUILD_COMMAND) CTEST_OUTPUT_ON_FAILURE=1 check

.PHONY: test
test: radar_test

.PHONY: clean
clean:
	rm -rf build
	rm -rf devel
	rm -rf install
	rm -rf src/bosch_radar/build
