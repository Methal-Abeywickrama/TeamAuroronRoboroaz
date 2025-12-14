CXX_SOURCES = maze_runner.cpp

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH ?= $(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))

include $(WEBOTS_HOME_PATH)/resources/Makefile.include
CXXFLAGS += -std=c++17