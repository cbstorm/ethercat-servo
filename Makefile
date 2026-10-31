# Convenience wrapper around CMake/CTest. CMake stays the real build system;
# every target here just calls cmake, ctest or a script in scripts/.
#
# Override any variable on the command line, e.g.
#   make build BUILD_TYPE=Debug WITH_IGH=ON ETHERCAT_ROOT=/opt/etherlab
#   make test TEST=Cia402

BUILD_DIR     ?= build
BUILD_TYPE    ?= RelWithDebInfo
JOBS          ?= $(shell nproc 2>/dev/null || echo 4)
PREFIX        ?= /usr/local
WITH_IGH      ?= AUTO
WITH_MQTT     ?= AUTO
WERROR        ?= OFF
ETHERCAT_ROOT ?=
CMAKE_ARGS    ?=
TEST          ?=
MASTER        ?= 0
ESI           ?= esi/LC-EH_V1.5.xml
CONFIG        ?= config/example.yaml
DIAG_SECONDS  ?= 10
ETHERCAT_TOOL ?= $(shell command -v ethercat 2>/dev/null || ls /opt/etherlab/bin/ethercat 2>/dev/null)

CMAKE_FLAGS := -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
               -DCMAKE_INSTALL_PREFIX=$(PREFIX) \
               -DETHERCAT_MOTION_WITH_IGH=$(WITH_IGH) \
               -DETHERCAT_MOTION_WITH_MQTT=$(WITH_MQTT) \
               -DETHERCAT_MOTION_WARNINGS_AS_ERRORS=$(WERROR)
ifneq ($(ETHERCAT_ROOT),)
CMAKE_FLAGS += -DEtherCAT_ROOT=$(ETHERCAT_ROOT)
endif
CMAKE_FLAGS += $(CMAKE_ARGS)

CTEST_FLAGS := --test-dir $(BUILD_DIR) --output-on-failure -j $(JOBS)
ifneq ($(TEST),)
CTEST_FLAGS += -R '$(TEST)'
endif

SANITIZE_FLAGS := -fsanitize=address,undefined -fno-omit-frame-pointer

.DEFAULT_GOAL := help
.PHONY: help setup deps deps-no-mqtt check-deps configure build rebuild debug release asan \
        test test-verbose test-list test-debug test-asan check run-sim run-multi run-c-api \
        scan scan-raw esi-list diagnose diagnose-live \
        install uninstall docs clean distclean

## ---- Setup ----------------------------------------------------------------

help: ## Show this help
	@awk 'BEGIN {FS = ":.*## "} \
	     /^## ----/ {sub(/^## -+ */, ""); sub(/ *-+$$/, ""); printf "\n\033[1m%s\033[0m\n", $$0} \
	     /^[a-zA-Z0-9_-]+:.*## / {printf "  \033[36m%-14s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)
	@printf '\nVariables: BUILD_DIR=%s BUILD_TYPE=%s JOBS=%s WITH_IGH=%s WITH_MQTT=%s PREFIX=%s\n' \
	        '$(BUILD_DIR)' '$(BUILD_TYPE)' '$(JOBS)' '$(WITH_IGH)' '$(WITH_MQTT)' '$(PREFIX)'

setup: deps configure ## Install apt dependencies (asks first, uses sudo) and configure

deps: ## Install apt build dependencies incl. MQTT (asks first, uses sudo)
	scripts/install_dependencies.sh

deps-no-mqtt: ## Install apt build dependencies without MQTT (asks first, uses sudo)
	scripts/install_dependencies.sh --no-mqtt

check-deps: ## Report which build/runtime dependencies are present (read-only)
	scripts/check_dependencies.sh

## ---- Build ----------------------------------------------------------------

configure: ## (Re)run CMake configure in BUILD_DIR
	cmake -S . -B $(BUILD_DIR) $(CMAKE_FLAGS)

$(BUILD_DIR)/CMakeCache.txt:
	cmake -S . -B $(BUILD_DIR) $(CMAKE_FLAGS)

build: $(BUILD_DIR)/CMakeCache.txt ## Build library, tests, examples and tools
	cmake --build $(BUILD_DIR) -j $(JOBS)

rebuild: ## Clean BUILD_DIR's build outputs and build again
	cmake --build $(BUILD_DIR) --target clean
	$(MAKE) build

debug: ## Debug build in build-debug/
	$(MAKE) build BUILD_DIR=build-debug BUILD_TYPE=Debug

release: ## Release build in build-release/
	$(MAKE) build BUILD_DIR=build-release BUILD_TYPE=Release

asan: ## Debug build with AddressSanitizer + UBSan in build-asan/
	$(MAKE) build BUILD_DIR=build-asan BUILD_TYPE=Debug \
	    CMAKE_ARGS='-DCMAKE_CXX_FLAGS="$(SANITIZE_FLAGS)" -DCMAKE_C_FLAGS="$(SANITIZE_FLAGS)" $(CMAKE_ARGS)'

## ---- Test -----------------------------------------------------------------

test: build ## Build and run the unit tests (TEST=<regex> to filter)
	ctest $(CTEST_FLAGS)

test-verbose: build ## Run the unit tests with full output
	ctest $(CTEST_FLAGS) -V

test-list: build ## List all discovered tests
	ctest --test-dir $(BUILD_DIR) -N

test-debug: ## Build and test the Debug configuration
	$(MAKE) test BUILD_DIR=build-debug BUILD_TYPE=Debug

test-asan: ## Build and test under AddressSanitizer + UBSan
	$(MAKE) test BUILD_DIR=build-asan BUILD_TYPE=Debug \
	    CMAKE_ARGS='-DCMAKE_CXX_FLAGS="$(SANITIZE_FLAGS)" -DCMAKE_C_FLAGS="$(SANITIZE_FLAGS)" $(CMAKE_ARGS)'

check: test ## Alias for test

## ---- Run (simulation, no hardware) ----------------------------------------

run-sim: build ## Run the simulation walkthrough example
	$(BUILD_DIR)/examples/simulation

run-multi: build ## Run the synchronized multi-axis example (simulation)
	$(BUILD_DIR)/examples/multi_axis

run-c-api: build ## Run the C API example
	$(BUILD_DIR)/examples/c_api_example

## ---- Hardware (read-only, never enables a drive) -------------------------

scan: build ## Scan the bus, match slaves to ESI, read 0x6502 (MASTER, ESI; needs IgH)
	$(BUILD_DIR)/tools/ethercat_motion_scan --master $(MASTER) --esi $(ESI)

scan-raw: ## List slaves with the IgH 'ethercat' tool (no build needed)
	@test -n "$(ETHERCAT_TOOL)" || { echo "IgH 'ethercat' tool not found - see docs/installation.md"; exit 1; }
	$(ETHERCAT_TOOL) master --master $(MASTER)
	$(ETHERCAT_TOOL) slaves --master $(MASTER) -v

esi-list: build ## List the devices described by the ESI file (ESI)
	$(BUILD_DIR)/tools/ethercat_motion_scan --esi $(ESI) --list

diagnose: build ## Validate CONFIG and print the effective PDO mapping
	$(BUILD_DIR)/tools/ethercat_motion_diagnostic $(CONFIG) --check

diagnose-live: build ## Run the cyclic task on CONFIG and print bus/drive/timing status
	$(BUILD_DIR)/tools/ethercat_motion_diagnostic $(CONFIG) --seconds $(DIAG_SECONDS)

## ---- Install / docs / clean -----------------------------------------------

install: build ## Install to PREFIX (use 'sudo make install' for system prefixes)
	cmake --install $(BUILD_DIR)

uninstall: ## Remove files listed in BUILD_DIR/install_manifest.txt
	@test -f $(BUILD_DIR)/install_manifest.txt || { echo "no $(BUILD_DIR)/install_manifest.txt - run 'make install' first"; exit 1; }
	xargs -d '\n' rm -f < $(BUILD_DIR)/install_manifest.txt

docs: ## Generate Doxygen API docs into build/docs (needs doxygen)
	@command -v doxygen >/dev/null || { echo "doxygen not found (sudo apt install doxygen graphviz)"; exit 1; }
	doxygen Doxyfile

clean: ## Remove build outputs in BUILD_DIR (keeps CMake cache)
	@if [ -f $(BUILD_DIR)/CMakeCache.txt ]; then cmake --build $(BUILD_DIR) --target clean; fi

distclean: ## Delete build/ and all build-*/ directories
	rm -rf build build-*/
