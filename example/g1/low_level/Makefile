# Makefile for motor_controller_with_enable using ZCAN API

CXX = g++
CXXFLAGS = -std=c++17 -Wall -O2
LDFLAGS = -lpthread

# ZLG CANFDNET SDK paths
ZLG_SDK_ROOT = /home/linaro/ludan_sdk/LINUX_CANFDNET_V1.3.0.5_example
ZLG_INCLUDE = -I$(ZLG_SDK_ROOT)/src/zlgcan
ZLG_LIB = -L$(ZLG_SDK_ROOT)/lib/linux_x64 -lCANFDNET

# Unitree SDK
UNITREE_INCLUDE = -I../../../../../include

# ROS2 include paths (manually added)
ROS2_INCLUDES = -I/opt/ros/humble/include

# Output executable
TARGET = motor_controller_with_enable

# Source files
SOURCES = motor_controller_with_enable.cpp

# Build rules
all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) $(ZLG_INCLUDE) $(UNITREE_INCLUDE) $(ROS2_INCLUDES) \
		-o $(TARGET) $(SOURCES) $(ZLG_LIB) $(LDFLAGS)

clean:
	rm -f $(TARGET)

.PHONY: all clean
