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

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#	include <winsock2.h>
#else /* !_WIN32 */
#	include <arpa/inet.h>
#endif /* !_WIN32 */

#include <json-c/json.h>
#include <metadata-thermal/tmeta.h>

#define ULOG_TAG tmeta
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#ifdef __APPLE__
#	include <machine/endian.h>
#elif defined(_WIN32)
#	define bswapll(y) (((uint64_t)ntohl(y)) << 32 | ntohl(y >> 32))
#	define htonll(y) bswapll(y)
#	define ntohll(y) bswapll(y)
#else
#	include <endian.h>
#	if __BYTE_ORDER == __LITTLE_ENDIAN
#		define bswapll(y) (((uint64_t)ntohl(y)) << 32 | ntohl(y >> 32))
#		define htonll(y) bswapll(y)
#		define ntohll(y) bswapll(y)
#	else
#		define htonll(y) (y)
#		define ntohll(y) (y)
#	endif
#endif


static const struct uuid {
	uint32_t uuid0;
	uint32_t uuid1;
	uint32_t uuid2;
	uint32_t uuid3;
} sei_uuid = {
	TMETA_USER_DATA_SEI_UUID_0,
	TMETA_USER_DATA_SEI_UUID_1,
	TMETA_USER_DATA_SEI_UUID_2,
	TMETA_USER_DATA_SEI_UUID_3,
};


static void serialize_thermal_metadata(const struct tmeta_data *meta, void *buf)
{
	uint32_t *pdw_buf = (uint32_t *)buf;
	*(pdw_buf++) = htonl(sei_uuid.uuid0);
	*(pdw_buf++) = htonl(sei_uuid.uuid1);
	*(pdw_buf++) = htonl(sei_uuid.uuid2);
	*(pdw_buf++) = htonl(sei_uuid.uuid3);

	uint8_t *pb_buf = (uint8_t *)pdw_buf;

	/* Ignore the version, force it to TMETA_VERSION */
	uint32_t tmp_u32;
	tmp_u32 = TMETA_VERSION;
	*(uint32_t *)pb_buf = htonl(tmp_u32);
	pb_buf += TMETA_VERSION_SIZE;

	/* V0.1 header data */
	tmp_u32 = meta->gain_mode;
	*(uint32_t *)pb_buf = htonl(tmp_u32);
	pb_buf += sizeof(tmp_u32);

	memcpy(pb_buf, &meta->calib_r, sizeof(double));
	pb_buf += sizeof(double);
	memcpy(pb_buf, &meta->calib_b, sizeof(double));
	pb_buf += sizeof(double);
	memcpy(pb_buf, &meta->calib_f, sizeof(double));
	pb_buf += sizeof(double);
	memcpy(pb_buf, &meta->calib_o, sizeof(double));
	pb_buf += sizeof(double);
	memcpy(pb_buf, &meta->calib_tau_win, sizeof(double));
	pb_buf += sizeof(double);
	memcpy(pb_buf, &meta->calib_t_win, sizeof(double));
	pb_buf += sizeof(double);
	memcpy(pb_buf, &meta->calib_t_bg, sizeof(double));
	pb_buf += sizeof(double);
	memcpy(pb_buf, &meta->calib_emissivity, sizeof(double));
	pb_buf += sizeof(double);

	*(uint32_t *)pb_buf = htonl(meta->jpeg_data_size);
	pb_buf += sizeof(meta->jpeg_data_size);

	*(uint32_t *)pb_buf = htonl((uint32_t)meta->value_min);
	pb_buf += sizeof(uint32_t);
	*(uint32_t *)pb_buf = htonl((uint32_t)meta->value_max);
	pb_buf += sizeof(uint32_t);

	memcpy(pb_buf, &meta->attitude_reference_quat, sizeof(float) * 4);
	pb_buf += sizeof(float) * 4;

	*(uint32_t *)pb_buf = htonl((uint32_t)meta->cam_angles_count);
	pb_buf += sizeof(uint32_t);

	/* V0.1 camera angles data */
	memcpy(pb_buf,
	       &meta->cam_angles,
	       sizeof(float) * meta->cam_angles_count * 4);
	pb_buf += sizeof(float) * meta->cam_angles_count * 4;

	unsigned int i;
	for (i = 0; i < meta->cam_angles_count; ++i) {
		*(uint64_t *)pb_buf =
			htonll((uint64_t)meta->cam_angles_timestamps[i]);
		pb_buf += sizeof(uint64_t);
	}

	/* V0.1 JPEG data */
	memcpy(pb_buf, meta->jpeg_data, meta->jpeg_data_size);
	pb_buf += meta->jpeg_data_size;

	/* V0.2 shutter state data */
	tmp_u32 = meta->frame_state;
	*(uint32_t *)pb_buf = htonl(tmp_u32);
	pb_buf += sizeof(uint32_t);

	/* V0.3 temperatures */
	memcpy(pb_buf, &meta->fpa_temp, sizeof(double));
	pb_buf += sizeof(double);
	memcpy(pb_buf, &meta->housing_temp, sizeof(double));
	pb_buf += sizeof(double);
	memcpy(pb_buf, &meta->window_reflection, sizeof(double));
	pb_buf += sizeof(double);

	/* V0.4 thermal camera alignment quaternion */
	memcpy(pb_buf, &meta->thermal_to_visible_quat, sizeof(float) * 4);
	pb_buf += sizeof(float) * 4;
}


static int deserialize_thermal_metadata(const void *buf,
					size_t buf_size,
					struct tmeta_data *meta)
{
	const uint8_t *pb_buf = (const uint8_t *)buf;
	unsigned int i;
	unsigned int cam_angles_size;
	unsigned int cam_angles_timestamps_size;

	ssize_t _buf_size = (ssize_t)buf_size;

	/* Check SEI UUID and version minimal buffer size */
	if (_buf_size < (ssize_t)(TMETA_SEI_UUID_SIZE + TMETA_VERSION_SIZE))
		return -1;

	/* Skip SEI UUID */
	pb_buf += TMETA_SEI_UUID_SIZE;
	_buf_size -= TMETA_SEI_UUID_SIZE;

	meta->version = ntohl(*(const uint32_t *)pb_buf);
	pb_buf += sizeof(uint32_t);
	_buf_size -= sizeof(uint32_t);

	if (TMETA_GET_MAJOR_VERSION(meta->version) > TMETA_MAJOR_VERSION) {
		/* Only Major version 0 is supported for now */
		return -1;
	}

	/* Check v0.1 header size */
	if (_buf_size < (ssize_t)TMETA_V0_1_HEADER_SIZE)
		return -1;

	/* Deserialize v0.1 header data */
	meta->gain_mode = ntohl(*(const uint32_t *)pb_buf);
	pb_buf += sizeof(uint32_t);
	_buf_size -= sizeof(uint32_t);

	memcpy(&meta->calib_r, pb_buf, sizeof(double));
	pb_buf += sizeof(double);
	_buf_size -= sizeof(double);
	memcpy(&meta->calib_b, pb_buf, sizeof(double));
	pb_buf += sizeof(double);
	_buf_size -= sizeof(double);
	memcpy(&meta->calib_f, pb_buf, sizeof(double));
	pb_buf += sizeof(double);
	_buf_size -= sizeof(double);
	memcpy(&meta->calib_o, pb_buf, sizeof(double));
	pb_buf += sizeof(double);
	_buf_size -= sizeof(double);
	memcpy(&meta->calib_tau_win, pb_buf, sizeof(double));
	pb_buf += sizeof(double);
	_buf_size -= sizeof(double);
	memcpy(&meta->calib_t_win, pb_buf, sizeof(double));
	pb_buf += sizeof(double);
	_buf_size -= sizeof(double);
	memcpy(&meta->calib_t_bg, pb_buf, sizeof(double));
	pb_buf += sizeof(double);
	_buf_size -= sizeof(double);
	memcpy(&meta->calib_emissivity, pb_buf, sizeof(double));
	pb_buf += sizeof(double);
	_buf_size -= sizeof(double);

	meta->jpeg_data_size = ntohl(*(const uint32_t *)pb_buf);
	pb_buf += sizeof(uint32_t);
	_buf_size -= sizeof(uint32_t);

	meta->value_min = ntohl(*(const uint32_t *)pb_buf);
	pb_buf += sizeof(uint32_t);
	_buf_size -= sizeof(uint32_t);
	meta->value_max = ntohl(*(const uint32_t *)pb_buf);
	pb_buf += sizeof(uint32_t);
	_buf_size -= sizeof(uint32_t);

	memcpy(&meta->attitude_reference_quat, pb_buf, sizeof(float) * 4);
	pb_buf += sizeof(float) * 4;
	_buf_size -= sizeof(float) * 4;

	meta->cam_angles_count = ntohl(*(const uint32_t *)pb_buf);
	pb_buf += sizeof(uint32_t);
	_buf_size -= sizeof(uint32_t);

	/* Check v0.1 camera angles size */
	cam_angles_size = sizeof(float) * meta->cam_angles_count * 4;
	cam_angles_timestamps_size = meta->cam_angles_count * sizeof(uint64_t);
	if (_buf_size < (ssize_t)(cam_angles_size + cam_angles_timestamps_size))
		return -1;

	memcpy(&meta->cam_angles, pb_buf, cam_angles_size);
	pb_buf += cam_angles_size;
	_buf_size -= cam_angles_size;
	for (i = 0; i < meta->cam_angles_count; ++i) {
		meta->cam_angles_timestamps[i] =
			ntohll(*(const uint64_t *)pb_buf);
		pb_buf += sizeof(uint64_t);
		_buf_size -= sizeof(uint64_t);
	}

	/* Check v0.1 JPEG data size */
	if (_buf_size < (ssize_t)meta->jpeg_data_size)
		return -1;
	meta->jpeg_data = (void *)pb_buf;
	pb_buf += meta->jpeg_data_size;
	_buf_size -= meta->jpeg_data_size;

	/* Stop here if minor version is lower than 2 */
	if (TMETA_GET_MINOR_VERSION(meta->version) < 2)
		goto end;
	if (_buf_size < (ssize_t)TMETA_V0_2_DATA_SIZE)
		return -1;

	meta->frame_state = ntohl(*(const uint32_t *)pb_buf);
	pb_buf += sizeof(uint32_t);
	_buf_size -= sizeof(uint32_t);

	/* Stop here if minor version is lower than 3 */
	if (TMETA_GET_MINOR_VERSION(meta->version) < 3)
		goto end;
	if (_buf_size < (ssize_t)TMETA_V0_3_DATA_SIZE)
		return -1;

	memcpy(&meta->fpa_temp, pb_buf, sizeof(double));
	pb_buf += sizeof(double);
	_buf_size -= sizeof(double);
	memcpy(&meta->housing_temp, pb_buf, sizeof(double));
	pb_buf += sizeof(double);
	_buf_size -= sizeof(double);
	memcpy(&meta->window_reflection, pb_buf, sizeof(double));
	pb_buf += sizeof(double);
	_buf_size -= sizeof(double);

	/* Stop here if minor version is lower than 4 */
	if (TMETA_GET_MINOR_VERSION(meta->version) < 4)
		goto end;
	if (_buf_size < (ssize_t)TMETA_V0_4_DATA_SIZE)
		return -1;

	memcpy(&meta->thermal_to_visible_quat, pb_buf, sizeof(float) * 4);
	pb_buf += sizeof(float) * 4;
	_buf_size -= sizeof(float) * 4;

end:
	return 0;
}


bool tmeta_is_thermal_metadata_user_data_sei(const void *buf, size_t buf_size)
{
	uint32_t uuid0, uuid1, uuid2, uuid3;
	const uint32_t *pdw_buf = (uint32_t *)buf;

	ULOG_ERRNO_RETURN_VAL_IF(buf == NULL, EINVAL, false);

	if (buf_size < (TMETA_SEI_UUID_SIZE + TMETA_VERSION_SIZE))
		return false;

	uuid0 = ntohl(*(pdw_buf++));
	uuid1 = ntohl(*(pdw_buf++));
	uuid2 = ntohl(*(pdw_buf++));
	uuid3 = ntohl(*(pdw_buf++));
	if (uuid0 != sei_uuid.uuid0 || uuid1 != sei_uuid.uuid1 ||
	    uuid2 != sei_uuid.uuid2 || uuid3 != sei_uuid.uuid3)
		return false;

	return true;
}


int tmeta_serialize_thermal_metadata_user_data_sei(
	const struct tmeta_data *meta,
	void *buf,
	size_t buf_size,
	size_t *size)
{
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	size_t _size = TMETA_BUF_SIZE(meta);
	if (buf_size < _size)
		return -ENOBUFS;

	serialize_thermal_metadata(meta, buf);

	if (size)
		*size = _size;

	return 0;
}


int tmeta_deserialize_thermal_metadata_user_data_sei(const void *buf,
						     size_t buf_size,
						     struct tmeta_data *meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	if (!tmeta_is_thermal_metadata_user_data_sei(buf, buf_size))
		return -ENOENT;

	return deserialize_thermal_metadata(buf, buf_size, meta);
}


enum tmeta_thermal_gain_mode tmeta_thermal_gain_mode_from_str(const char *str)
{
	if (strcasecmp(str, "FLIR_LOW_GAIN") == 0) {
		return TMETA_THERMAL_GAIN_MODE_FLIR_LOW_GAIN;
	} else if (strcasecmp(str, "FLIR_HIGH_GAIN") == 0) {
		return TMETA_THERMAL_GAIN_MODE_FLIR_HIGH_GAIN;
	} else {
		ULOGW("%s: unknown gain mode '%s'", __func__, str);
		return TMETA_THERMAL_GAIN_MODE_FLIR_LOW_GAIN;
	}
}


const char *tmeta_thermal_gain_mode_to_str(enum tmeta_thermal_gain_mode mode)
{
	switch (mode) {
	case TMETA_THERMAL_GAIN_MODE_FLIR_LOW_GAIN:
		return "FLIR_LOW_GAIN";
	case TMETA_THERMAL_GAIN_MODE_FLIR_HIGH_GAIN:
		return "FLIR_HIGH_GAIN";
	default:
		return "UNKNOWN";
	}
}


enum tmeta_thermal_frame_state
tmeta_thermal_frame_state_from_str(const char *str)
{
	if (strcasecmp(str, "VALID") == 0) {
		return TMETA_THERMAL_FRAME_STATE_VALID;
	} else if (strcasecmp(str, "SHUTTER_DESIRED") == 0) {
		return TMETA_THERMAL_FRAME_STATE_SHUTTER_DESIRED;
	} else if (strcasecmp(str, "SHUTTER_IN_PROGRESS") == 0) {
		return TMETA_THERMAL_FRAME_STATE_SHUTTER_IN_PROGRESS;
	} else if (strcasecmp(str, "UNEXPECTED") == 0) {
		return TMETA_THERMAL_FRAME_STATE_UNEXPECTED;
	} else {
		ULOGW("%s: unknown frame state '%s'", __func__, str);
		return TMETA_THERMAL_FRAME_STATE_UNEXPECTED;
	}
}


const char *
tmeta_thermal_frame_state_to_str(enum tmeta_thermal_frame_state state)
{
	switch (state) {
	case TMETA_THERMAL_FRAME_STATE_VALID:
		return "VALID";
	case TMETA_THERMAL_FRAME_STATE_SHUTTER_DESIRED:
		return "SHUTTER_DESIRED";
	case TMETA_THERMAL_FRAME_STATE_SHUTTER_IN_PROGRESS:
		return "SHUTTER_IN_PROGRESS";
	case TMETA_THERMAL_FRAME_STATE_UNEXPECTED:
		return "UNEXPECTED";
	default:
		return "UNKNOWN";
	}
}


static struct json_object *tmeta_json_object_new_quaternion(const float quat[4])
{
	struct json_object *jobj_val = json_object_new_object();

	json_object_object_add(jobj_val, "x", json_object_new_double(quat[0]));
	json_object_object_add(jobj_val, "y", json_object_new_double(quat[1]));
	json_object_object_add(jobj_val, "z", json_object_new_double(quat[2]));
	json_object_object_add(jobj_val, "w", json_object_new_double(quat[3]));

	return jobj_val;
}


int tmeta_thermal_metadata_to_json(const struct tmeta_data *meta,
				   struct json_object *jobj)
{
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(jobj == NULL, EINVAL);

	/* Structure format version (major number and minor number) */
	json_object_object_add(
		jobj,
		"version_major",
		json_object_new_int(TMETA_GET_MAJOR_VERSION(meta->version)));
	json_object_object_add(
		jobj,
		"version_minor",
		json_object_new_int(TMETA_GET_MINOR_VERSION(meta->version)));

	/* Active gain mode for this frame */
	json_object_object_add(
		jobj,
		"gain_mode",
		json_object_new_string(
			tmeta_thermal_gain_mode_to_str(meta->gain_mode)));

	/* R calibration value for this frame */
	json_object_object_add(
		jobj, "calib_r", json_object_new_double(meta->calib_r));

	/* B calibration value for this frame */
	json_object_object_add(
		jobj, "calib_b", json_object_new_double(meta->calib_b));

	/* F calibration value for this frame */
	json_object_object_add(
		jobj, "calib_f", json_object_new_double(meta->calib_f));

	/* O calibration value for this frame */
	json_object_object_add(
		jobj, "calib_o", json_object_new_double(meta->calib_o));

	/* tauWin calibration value for this frame */
	json_object_object_add(jobj,
			       "calib_tau_win",
			       json_object_new_double(meta->calib_tau_win));

	/* tWin calibration value for this frame */
	json_object_object_add(
		jobj, "calib_t_win", json_object_new_double(meta->calib_t_win));

	/* tBg calibration value for this frame */
	json_object_object_add(
		jobj, "calib_t_bg", json_object_new_double(meta->calib_t_bg));

	/* Emissivity calibration value for this frame */
	json_object_object_add(jobj,
			       "calib_emissivity",
			       json_object_new_double(meta->calib_emissivity));

	/* Size in bytes of the JPEG data */
	json_object_object_add(jobj,
			       "jpeg_data_size",
			       json_object_new_int(meta->jpeg_data_size));

	/* Mininmum raw thermal value for this frame */
	json_object_object_add(
		jobj, "value_min", json_object_new_int(meta->value_min));

	/* Maximum raw thermal value for this frame */
	json_object_object_add(
		jobj, "value_max", json_object_new_int(meta->value_max));

	/* Drone attitude reference quaternion (x, y, z, w) */
	json_object_object_add(jobj,
			       "attitude_reference_quat",
			       tmeta_json_object_new_quaternion(
				       meta->attitude_reference_quat));

	/* Camera angles quaternions (x, y, z, w) and timestamps */
	struct json_object *jcam_angles = json_object_new_array();
	if (jcam_angles == NULL) {
		int res = -ENOMEM;
		ULOG_ERRNO("json_object_new_array", -res);
		return res;
	}
	struct json_object *jcam_angles_timestamps = json_object_new_array();
	if (jcam_angles_timestamps == NULL) {
		int res = -ENOMEM;
		ULOG_ERRNO("json_object_new_array", -res);
		json_object_put(jcam_angles);
		return res;
	}
	for (uint32_t i = 0; i < meta->cam_angles_count; i++) {
		json_object_array_add(jcam_angles,
				      tmeta_json_object_new_quaternion(
					      meta->cam_angles + i * 4));
		json_object_array_add(
			jcam_angles_timestamps,
			json_object_new_int64(meta->cam_angles_timestamps[i]));
	}
	json_object_object_add(jobj, "cam_angles", jcam_angles);
	json_object_object_add(
		jobj, "cam_angles_timestamps", jcam_angles_timestamps);

	/* Thermal shutter state */
	json_object_object_add(
		jobj,
		"frame_state",
		json_object_new_string(
			tmeta_thermal_frame_state_to_str(meta->frame_state)));

	/* Temperature of the focal plane array */
	json_object_object_add(
		jobj, "fpa_temp", json_object_new_double(meta->fpa_temp));

	/* Temperature measured by the housing thermistor */
	json_object_object_add(jobj,
			       "housing_temp",
			       json_object_new_double(meta->housing_temp));

	/* Window reflected temperature */
	json_object_object_add(jobj,
			       "window_reflection",
			       json_object_new_double(meta->window_reflection));

	/* Thermal camera alignment quaternion (x, y, z, w) */
	json_object_object_add(jobj,
			       "thermal_to_visible_quat",
			       tmeta_json_object_new_quaternion(
				       meta->thermal_to_visible_quat));

	return 0;
}
