LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libmetadata-thermal
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Parrot Drones thermal metadata library
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
# Public API headers - top level headers first
# This header list is currently used to generate a python binding
LOCAL_EXPORT_CUSTOM_VARIABLES := LIBMETADATATHERMAL_HEADERS=$\
	$(LOCAL_PATH)/include/metadata-thermal/tmeta.h;

LOCAL_CFLAGS := -DTMETA_API_EXPORTS -fvisibility=hidden -std=gnu99

LOCAL_SRC_FILES := \
	src/tmeta.c

LOCAL_PRIVATE_LIBRARIES := \
	json \
	libulog

ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_LIBRARY)
