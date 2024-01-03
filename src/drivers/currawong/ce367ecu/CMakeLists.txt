############################################################################
#
#   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
#add_subdirectory(piccolo_protocol)

px4_add_module(
	MODULE drivers__currawong__ce367ecu
	MAIN ce367ecu
	COMPILE_FLAGS
		-O0
	SRCS
		ce367ecu_main.cpp
		ce367ecu_can.cpp
		ce367ecu_serial.cpp
		piccolo_protocol/fielddecode.cpp
		piccolo_protocol/fieldencode.cpp
		piccolo_protocol/scaleddecode.cpp
		piccolo_protocol/scaledencode.cpp
		piccolo_protocol/ECUDefines.cpp
		piccolo_protocol/ECUProtocol.cpp
		piccolo_protocol/ECUPackets.cpp
		piccolo_protocol/ECUSettings.cpp
		piccolo_protocol/PMUDefines.cpp
		piccolo_protocol/PMUProtocol.cpp
		piccolo_protocol/PMUPackets.cpp
		piccolo_protocol/PMUSettings.cpp

	MODULE_CONFIG
		module.yaml
	DEPENDS
	)
