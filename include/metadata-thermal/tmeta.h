/**
 * Copyright (c) 2017 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Parrot Drones SAS Company nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT DRONES SAS COMPANY BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _TMETA_H_
#define _TMETA_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>

/* To be used for all public API */
#ifdef TMETA_API_EXPORTS
#	ifdef _WIN32
#		define TMETA_API __declspec(dllexport)
#	else /* !_WIN32 */
#		define TMETA_API __attribute__((visibility("default")))
#	endif /* !_WIN32 */
#else /* !TMETA_API_EXPORTS */
#	define TMETA_API
#endif /* !TMETA_API_EXPORTS */


/* Forward declaration */
struct json_object;


/* codecheck_ignore_file[LONG_LINE] */
/* clang-format off */
/**
 * Global data structure
 *
 * +---+---+------------------------------------------------------+----------+-----------+------+
 * | U | V |                                                      |          |           |      |
 * | U | E |                    V0.1                              |   V0.2   |   V0.3    | V0.4 |
 * | I | R |                                                      |          |           |      |
 * | D | S +-----------------------+------------------------------+----------+-----------+------+
 * |   | I |                       |                              |          |           |   T  |
 * | S | O |     HEADER            |      DATA (VARIABLE SIZE)    | SHUTTER  |   TEMP    |   H  |
 * | E | N |                       |                              |  STATE   |           |   E  |
 * | I |   +---+---+---+---+---+---+-----------+------------------+----------+---+---+---+   R  |
 * |   |   | G | C | J | M | A | D |           |                  |          | F | H | W |   M  |
 * |   |   | A | A | P | I | T | A |           |                  |          | P | O | I |   A  |
 * |   |   | I | L | G | N | T | T | TELEMETRY |   JPG THERMAL    |          | A | U | N |   L  |
 * |   |   | N | I |   | / | I | A |           |                  |          |   | S | D |      |
 * |   |   |   | B | D | M | T |   |           |                  |          | T | I | O |   C  |
 * |   |   | M | R | A | A | U | C +-------+---+------------------+          | E | N | W |   A  |
 * |   |   | O | A | T | X | D | O |   C   | T |                             | M | G |   |   M  |
 * |   |   | D | T | A |   | E | U |   A   | I |                             | P |   | R |   E  |
 * |   |   | E | I |   | T |   | N |   M   | M |                             |   | T | E |   R  |
 * |   |   |   | O | S | E |   | T |       | E |                             |   | E | F |   A  |
 * |   |   |   | N | I | M |   |   |   A   | S |                             |   | M | L |      |
 * |   |   |   |   | Z | P |   |   |   N   | T |                             |   | P | E |   A  |
 * |   |   |   |   | E |   |   |   |   G   | A |                             |   |   | C |   L  |
 * |   |   |   |   |   |   |   |   |   L   | M |                             |   |   | T |   I  |
 * |   |   |   |   |   |   |   |   |   E   | P |                             |   |   | I |   G  |
 * |   |   |   |   |   |   |   |   |   S   | S |                             |   |   | O |   N  |
 * +---+---+---+---+---+---+---+---+-------+---+                             |   |   | N |      |
 *                                                                           +---+---+---+------+
 */
/* clang-format on */


/* User data SEI UUID size */
#define TMETA_SEI_UUID_SIZE (4 * sizeof(uint32_t))

/* Version size composed of major and minor numbers */
#define TMETA_VERSION_SIZE sizeof(uint32_t)

/* Fixed header size for version 0.1 of the structure */
#define TMETA_V0_1_HEADER_SIZE                                                 \
	(sizeof(uint32_t) /* gain_mode */ +                                    \
	 sizeof(double) * 8 /* calibration data */ +                           \
	 sizeof(uint32_t) /* JPEG data size */ +                               \
	 sizeof(uint32_t) * 2 /* min/max temperatures */ +                     \
	 sizeof(float) * 4 /* attitude reference */ +                          \
	 sizeof(uint32_t) /* camera angles count */)

/* Version 0.2 added size */
#define TMETA_V0_2_DATA_SIZE sizeof(uint32_t) /* shutter state */

/* Version 0.3 added size */
#define TMETA_V0_3_DATA_SIZE (3 * sizeof(double)) /* temperatures */

/* Version 0.4 added size */
#define TMETA_V0_4_DATA_SIZE (4 * sizeof(float)) /* thermal cam alignment */

/* Total buffer size */
#define TMETA_BUF_SIZE(meta)                                                   \
	(TMETA_SEI_UUID_SIZE + TMETA_VERSION_SIZE + TMETA_V0_1_HEADER_SIZE +   \
	 sizeof(float) * (meta)->cam_angles_count * 4 +                        \
	 sizeof(uint64_t) * (meta)->cam_angles_count +                         \
	 (meta)->jpeg_data_size + TMETA_V0_2_DATA_SIZE +                       \
	 TMETA_V0_3_DATA_SIZE + TMETA_V0_4_DATA_SIZE)


/* Current version major number */
#define TMETA_MAJOR_VERSION 0x0

/* Current version minor number */
#define TMETA_MINOR_VERSION 0x4

/* Full version as 32bit value */
#define TMETA_VERSION (TMETA_MAJOR_VERSION << 16 | TMETA_MINOR_VERSION)

/* Get the version major and minor numbers */
#define TMETA_GET_MAJOR_VERSION(version) ((version >> 16) & 0x0FFFF)
#define TMETA_GET_MINOR_VERSION(version) (version & 0x0FFFF)


/* Maximum number of camera angles in the structure */
#define TMETA_CAMANGLES_MAXCOUNT 50


/* Thermal gain mode */
enum tmeta_thermal_gain_mode {
	/* FLIR low gain mode */
	TMETA_THERMAL_GAIN_MODE_FLIR_LOW_GAIN = 0,

	/* FLIR high gain mode */
	TMETA_THERMAL_GAIN_MODE_FLIR_HIGH_GAIN = 1,
};


/* Thermal frame state */
enum tmeta_thermal_frame_state {
	/* Valid thermal frame */
	TMETA_THERMAL_FRAME_STATE_VALID = 0,

	/* Shutter pending */
	TMETA_THERMAL_FRAME_STATE_SHUTTER_DESIRED,

	/* Shutter in progress */
	TMETA_THERMAL_FRAME_STATE_SHUTTER_IN_PROGRESS,

	/* Unexpected state */
	TMETA_THERMAL_FRAME_STATE_UNEXPECTED,
};


/* Thermal metadata */
struct tmeta_data {
	/* Version 0.1 base */

	/* Structure format version (major number as high 16 bits, minor number
	 * as low 16 bits) */
	uint32_t version;

	/* Active gain mode for this frame, serialized as a uint32_t value */
	enum tmeta_thermal_gain_mode gain_mode;

	/* R calibration value for this frame */
	double calib_r;

	/* B calibration value for this frame */
	double calib_b;

	/* F calibration value for this frame */
	double calib_f;

	/* O calibration value for this frame */
	double calib_o;

	/* tauWin calibration value for this frame */
	double calib_tau_win;

	/* tWin calibration value for this frame */
	double calib_t_win;

	/* tBg calibration value for this frame */
	double calib_t_bg;

	/* Emissivity calibration value for this frame */
	double calib_emissivity;

	/* Size in bytes of the JPEG data */
	uint32_t jpeg_data_size;

	/* Mininmum raw thermal value for this frame */
	uint32_t value_min;

	/* Maximum raw thermal value for this frame */
	uint32_t value_max;

	/* Drone attitude reference quaternion (x, y, z, w) */
	float attitude_reference_quat[4];

	/* Camera angles count */
	uint32_t cam_angles_count;

	/* Camera angles quaternions (x, y, z, w) */
	float cam_angles[TMETA_CAMANGLES_MAXCOUNT * 4];

	/* Camera angles timestamps in microseconds */
	uint64_t cam_angles_timestamps[TMETA_CAMANGLES_MAXCOUNT];

	/* Pointer to the scaled raw thermal values encoded as an
	 * 8bit JPEG image */
	void *jpeg_data;

	/* Added in version 0.2 */

	/* Thermal shutter state */
	enum tmeta_thermal_frame_state frame_state;

	/* Added in version 0.3 */

	/* Temperature of the focal plane array */
	double fpa_temp;

	/* Temperature measured by the housing thermistor */
	double housing_temp;

	/* Window reflected temperature */
	double window_reflection;

	/* Added in version 0.4 */

	/* Thermal camera alignment quaternion (x, y, z, w) */
	float thermal_to_visible_quat[4];
};


/* Thermal metadata user data SEI UUID;
 * UUID: a4897b82-4415-4171-b46a-bc8cd524c77e */
#define TMETA_USER_DATA_SEI_UUID_0 0xa4897b82
#define TMETA_USER_DATA_SEI_UUID_1 0x44154171
#define TMETA_USER_DATA_SEI_UUID_2 0xb46abc8c
#define TMETA_USER_DATA_SEI_UUID_3 0xd524c77e


/**
 * Is this a thermal metadata user data SEI?
 * @param buf: pointer to the metadata buffer
 * @param buf_size: size in bytes of the metadata buffer
 * @return true if the user data SEI is recognized as thermal metadata,
 * false otherwise
 */
TMETA_API
bool tmeta_is_thermal_metadata_user_data_sei(const void *buf, size_t buf_size);


/**
 * Serialize a thermal metadata user data SEI.
 * The function parses a thermal metadata structure and fills the user data
 * SEI buffer. buf_size must be at least TMETA_BUF_SIZE(meta).
 * @param meta: pointer to the thermal metadata structure
 * @param buf: pointer to the user data SEI buffer to fill (output)
 * @param buf_size: size in bytes of the user data SEI buffer
 * @param size: pointer to the final user data SEI size in bytes (output)
 * @return 0 on success, negative errno value in case of error
 */
TMETA_API
int tmeta_serialize_thermal_metadata_user_data_sei(
	const struct tmeta_data *meta,
	void *buf,
	size_t buf_size,
	size_t *size);


/**
 * Deserialize a thermal metadata user data SEI.
 * The function parses a thermal metadata user data SEI and fills the
 * thermal metadata structure.
 * @param buf: pointer to the user data SEI buffer
 * @param buf_size: size in bytes of the user data SEI
 * @param meta: pointer to the thermal metadata structure to fill (output)
 * @return 0 on success, negative errno value in case of error
 */
TMETA_API
int tmeta_deserialize_thermal_metadata_user_data_sei(const void *buf,
						     size_t buf_size,
						     struct tmeta_data *meta);


/**
 * Get an enum tmeta_thermal_gain_mode value from a string.
 * Valid strings are only the suffix of the gain mode name
 * (eg. 'FLIR_LOW_GAIN'). The case is ignored.
 * @param str: gain mode name to convert
 * @return the enum tmeta_thermal_gain_mode value or
 *         TMETA_THERMAL_GAIN_MODE_FLIR_LOW_GAIN if unknown
 */
TMETA_API enum tmeta_thermal_gain_mode
tmeta_thermal_gain_mode_from_str(const char *str);


/**
 * Get a string from an enum tmeta_thermal_gain_mode value.
 * @param mode: gain mode value to convert
 * @return a string description of the gain mode
 */
TMETA_API const char *
tmeta_thermal_gain_mode_to_str(enum tmeta_thermal_gain_mode mode);


/**
 * Get an enum tmeta_thermal_frame_state value from a string.
 * Valid strings are only the suffix of the frame state name
 * (eg. 'SHUTTER_IN_PROGRESS'). The case is ignored.
 * @param str: frame state name to convert
 * @return the enum tmeta_thermal_frame_state value or
 *         TMETA_THERMAL_FRAME_STATE_UNEXPECTED if unknown
 */
TMETA_API enum tmeta_thermal_frame_state
tmeta_thermal_frame_state_from_str(const char *str);


/**
 * Get a string from an enum tmeta_thermal_frame_state value.
 * @param state: frame state value to convert
 * @return a string description of the frame state
 */
TMETA_API const char *
tmeta_thermal_frame_state_to_str(enum tmeta_thermal_frame_state state);


/**
 * Write thermal metadata to a JSON object.
 * The jobj JSON object must have been previously allocated.
 * The ownership of the JSON object stays with the caller.
 * @param meta: pointer to a thermal metadata structure
 * @param jobj: pointer to the JSON object to write to (output)
 * @return 0 on success, negative errno value in case of error
 */
TMETA_API
int tmeta_thermal_metadata_to_json(const struct tmeta_data *meta,
				   struct json_object *jobj);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_TMETA_H_ */
