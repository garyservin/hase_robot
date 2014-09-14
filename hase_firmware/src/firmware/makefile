# Copyright 2011 Adam Green (http://mbed.org/users/AdamGreen/)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
PROJECT         := haseROS
DEVICES         := LPC1768
GCC4MBED_DIR    := /home/gary/src/external/mbed/gcc4mbed
INCDIRS         := libraries
NO_FLOAT_SCANF  := 1

ifdef LPC_RESET
reset:
	@echo "Resetting the target"
	$(LPC_RESET)
endif

include $(GCC4MBED_DIR)/build/gcc4mbed.mk
