############################################################################
#
# Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# Help
# --------------------------------------------------------------------
# Don't be afraid of this makefile, it is just passing
# arguments to cmake to allow us to keep the wiki pages etc.
# that describe how to build the px4 firmware
# the same even when using cmake instead of make.
#
# Example usage:
#
# make px4fmu-v2_default 			(builds)
# make px4fmu-v2_default upload 	(builds and uploads)
# make px4fmu-v2_default test 		(builds and tests)
#
# This tells cmake to build the nuttx px4fmu-v2 default config in the
# directory build_nuttx_px4fmu-v2_default and then call make
# in that directory with the target upload.

# Parsing
# --------------------------------------------------------------------
# assume 1st argument passed is the main target, the
# rest are arguments to pass to the makefile generated
# by cmake in the subdirectory
ARGS := $(wordlist 2,$(words $(MAKECMDGOALS)),$(MAKECMDGOALS))
j ?= 4

# Functions
# --------------------------------------------------------------------
# describe how to build a cmake config
define cmake-build
+mkdir -p $(PWD)/build_$@ && cd $(PWD)/build_$@ && cmake .. -DCONFIG=$(1)
+make -j$(j) -C $(PWD)/build_$@ --no-print-directory $(ARGS)
endef

# create empty targets to avoid msgs for targets passed to cmake
define cmake-targ
$(1):
	@#
.PHONY: $(1)
endef

# ADD CONFIGS HERE
# --------------------------------------------------------------------
#  Do not put any spaces between function arguments.

px4fmu-v2_default:
	$(call cmake-build,nuttx_px4fmu-v2_default)

px4fmu-v2_simple:
	$(call cmake-build,nuttx_px4fmu-v2_simple)

nuttx_sim_simple:
	$(call cmake-build,$@)

posix_sitl_simple:
	$(call cmake-build,$@)

ros_sitl_simple:
	$(call cmake-build,$@)

qurt_eagle_travis:
	$(call cmake-build,$@)

posix: posix_sitl_simple

ros: ros_sitl_simple

run_sitl_quad: posix
	Tools/sitl_run.sh posix-configs/SITL/init/rcS

run_sitl_plane: posix
	Tools/sitl_run.sh posix-configs/SITL/init/rc.fixed_wing

run_sitl_ros: posix
	Tools/sitl_run.sh posix-configs/SITL/init/rc_iris_ros

lldb_sitl_quad: posix
	Tools/sitl_run.sh posix-configs/SITL/init/rcS lldb

lldb_sitl_plane: posix
	Tools/sitl_run.sh posix-configs/SITL/init/rc.fixed_wing lldb

lldb_sitl_ros: posix
	Tools/sitl_run.sh posix-configs/SITL/init/rc_iris_ros lldb

gdb_sitl_quad: posix
	Tools/sitl_run.sh posix-configs/SITL/init/rcS gdb

gdb_sitl_plane: posix
	Tools/sitl_run.sh posix-configs/SITL/init/rc.fixed_wing lldb

gdb_sitl_ros: posix
	Tools/sitl_run.sh posix-configs/SITL/init/rc_iris_ros lldb

sitl_quad:
	@echo "Deprecated. Use 'run_sitl_quad' instead."

sitl_plane:
	@echo "Deprecated. Use 'run_sitl_plane' instead."

sitl_ros:
	@echo "Deprecated. Use 'run_sitl_ros' instead."

# Other targets
# --------------------------------------------------------------------
#  explicity set default build target
all: px4fmu-v2_default

clean:
	rm -rf build_*/

# targets handled by cmake
cmake_targets = test upload package package_source debug debug_io check_weak
$(foreach targ,$(cmake_targets),$(eval $(call cmake-targ,$(targ))))

.PHONY: clean

CONFIGS:=$(shell ls cmake/configs | sed -e "s~.*/~~" | sed -e "s~\..*~~")

# Future:
#$(CONFIGS):
##	@cd Build/$@ && cmake ../.. -DCONFIG=$@
#	@cd Build/$@ && make
#
#clean-all:
#	@rm -rf Build/*
#
#help:
#	@echo
#	@echo "Type 'make ' and hit the tab key twice to see a list of the available"
#	@echo "build configurations."
#	@echo
