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

#include "tmeta_test.h"

#include <json-c/json.h>


static struct json_object *build_json_for(struct tmeta_data *meta)
{
	struct json_object *jobj = json_object_new_object();
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(jobj);
	res = tmeta_thermal_metadata_to_json(meta, jobj);
	CU_ASSERT_EQUAL(res, 0);
	return jobj;
}


static double get_double(struct json_object *jobj, const char *key)
{
	struct json_object *val = NULL;

	CU_ASSERT_TRUE_FATAL(json_object_object_get_ex(jobj, key, &val));
	return json_object_get_double(val);
}


static int get_int(struct json_object *jobj, const char *key)
{
	struct json_object *val = NULL;

	CU_ASSERT_TRUE_FATAL(json_object_object_get_ex(jobj, key, &val));
	return json_object_get_int(val);
}


static const char *get_string(struct json_object *jobj, const char *key)
{
	struct json_object *val = NULL;

	CU_ASSERT_TRUE_FATAL(json_object_object_get_ex(jobj, key, &val));
	return json_object_get_string(val);
}


static void check_quaternion(struct json_object *jobj,
			     const char *key,
			     const float *expected)
{
	struct json_object *val = NULL;
	struct json_object *comp = NULL;

	CU_ASSERT_TRUE_FATAL(json_object_object_get_ex(jobj, key, &val));
	CU_ASSERT_TRUE(json_object_object_get_ex(val, "x", &comp));
	CU_ASSERT_EQUAL(json_object_get_double(comp), (double)expected[0]);
	CU_ASSERT_TRUE(json_object_object_get_ex(val, "y", &comp));
	CU_ASSERT_EQUAL(json_object_get_double(comp), (double)expected[1]);
	CU_ASSERT_TRUE(json_object_object_get_ex(val, "z", &comp));
	CU_ASSERT_EQUAL(json_object_get_double(comp), (double)expected[2]);
	CU_ASSERT_TRUE(json_object_object_get_ex(val, "w", &comp));
	CU_ASSERT_EQUAL(json_object_get_double(comp), (double)expected[3]);
}


static void test_to_json_null_meta(void)
{
	struct json_object *jobj = json_object_new_object();

	CU_ASSERT_PTR_NOT_NULL_FATAL(jobj);
	CU_ASSERT_EQUAL(tmeta_thermal_metadata_to_json(NULL, jobj), -EINVAL);
	json_object_put(jobj);
}


static void test_to_json_null_jobj(void)
{
	struct tmeta_data meta;
	uint8_t dummy = 0;

	fill_tmeta_data(&meta, 0, 0, &dummy);

	CU_ASSERT_EQUAL(tmeta_thermal_metadata_to_json(&meta, NULL), -EINVAL);
}


static void test_to_json_scalar_fields(void)
{
	struct tmeta_data meta;
	struct json_object *jobj;
	uint8_t dummy = 0;

	fill_tmeta_data(&meta, 0, 0, &dummy);
	jobj = build_json_for(&meta);

	CU_ASSERT_EQUAL(get_int(jobj, "version_major"),
			(int)TMETA_GET_MAJOR_VERSION(meta.version));
	CU_ASSERT_EQUAL(get_int(jobj, "version_minor"),
			(int)TMETA_GET_MINOR_VERSION(meta.version));
	CU_ASSERT_STRING_EQUAL(get_string(jobj, "gain_mode"), "FLIR_HIGH_GAIN");
	CU_ASSERT_EQUAL(get_double(jobj, "calib_r"), meta.calib_r);
	CU_ASSERT_EQUAL(get_double(jobj, "calib_b"), meta.calib_b);
	CU_ASSERT_EQUAL(get_double(jobj, "calib_f"), meta.calib_f);
	CU_ASSERT_EQUAL(get_double(jobj, "calib_o"), meta.calib_o);
	CU_ASSERT_EQUAL(get_double(jobj, "calib_tau_win"), meta.calib_tau_win);
	CU_ASSERT_EQUAL(get_double(jobj, "calib_t_win"), meta.calib_t_win);
	CU_ASSERT_EQUAL(get_double(jobj, "calib_t_bg"), meta.calib_t_bg);
	CU_ASSERT_EQUAL(get_double(jobj, "calib_emissivity"),
			meta.calib_emissivity);
	CU_ASSERT_EQUAL(get_int(jobj, "jpeg_data_size"),
			(int)meta.jpeg_data_size);
	CU_ASSERT_EQUAL(get_int(jobj, "value_min"), (int)meta.value_min);
	CU_ASSERT_EQUAL(get_int(jobj, "value_max"), (int)meta.value_max);
	CU_ASSERT_STRING_EQUAL(get_string(jobj, "frame_state"),
			       "SHUTTER_IN_PROGRESS");
	CU_ASSERT_EQUAL(get_double(jobj, "fpa_temp"), meta.fpa_temp);
	CU_ASSERT_EQUAL(get_double(jobj, "housing_temp"), meta.housing_temp);
	CU_ASSERT_EQUAL(get_double(jobj, "window_reflection"),
			meta.window_reflection);

	json_object_put(jobj);
}


static void test_to_json_quaternions(void)
{
	struct tmeta_data meta;
	struct json_object *jobj;
	uint8_t dummy = 0;

	fill_tmeta_data(&meta, 0, 0, &dummy);
	jobj = build_json_for(&meta);

	check_quaternion(
		jobj, "attitude_reference_quat", meta.attitude_reference_quat);
	check_quaternion(
		jobj, "thermal_to_visible_quat", meta.thermal_to_visible_quat);

	json_object_put(jobj);
}


static void test_to_json_cam_angles_arrays(void)
{
	struct tmeta_data meta;
	struct json_object *jobj;
	struct json_object *jangles = NULL;
	struct json_object *jtimestamps = NULL;
	struct json_object *elem;
	struct json_object *comp = NULL;
	uint32_t i;

	fill_tmeta_data(&meta, 2, 0, NULL);
	jobj = build_json_for(&meta);

	CU_ASSERT_TRUE_FATAL(
		json_object_object_get_ex(jobj, "cam_angles", &jangles));
	CU_ASSERT_TRUE_FATAL(json_object_object_get_ex(
		jobj, "cam_angles_timestamps", &jtimestamps));
	CU_ASSERT_EQUAL(json_object_array_length(jangles), 2);
	CU_ASSERT_EQUAL(json_object_array_length(jtimestamps), 2);

	for (i = 0; i < 2; i++) {
		elem = json_object_array_get_idx(jangles, i);
		CU_ASSERT_PTR_NOT_NULL_FATAL(elem);
		CU_ASSERT_TRUE(json_object_object_get_ex(elem, "x", &comp));
		CU_ASSERT_EQUAL(json_object_get_double(comp),
				(double)meta.cam_angles[i * 4 + 0]);

		elem = json_object_array_get_idx(jtimestamps, i);
		CU_ASSERT_PTR_NOT_NULL_FATAL(elem);
		CU_ASSERT_EQUAL((uint64_t)json_object_get_int64(elem),
				meta.cam_angles_timestamps[i]);
	}

	json_object_put(jobj);
}


static void test_to_json_zero_cam_angles(void)
{
	struct tmeta_data meta;
	struct json_object *jobj;
	struct json_object *jangles = NULL;
	struct json_object *jtimestamps = NULL;
	uint8_t dummy = 0;

	fill_tmeta_data(&meta, 0, 0, &dummy);
	jobj = build_json_for(&meta);

	CU_ASSERT_TRUE_FATAL(
		json_object_object_get_ex(jobj, "cam_angles", &jangles));
	CU_ASSERT_TRUE_FATAL(json_object_object_get_ex(
		jobj, "cam_angles_timestamps", &jtimestamps));
	CU_ASSERT_EQUAL(json_object_array_length(jangles), 0);
	CU_ASSERT_EQUAL(json_object_array_length(jtimestamps), 0);

	json_object_put(jobj);
}


CU_TestInfo s_json_tests[] = {
	{FN("null meta"), &test_to_json_null_meta},
	{FN("null jobj"), &test_to_json_null_jobj},
	{FN("scalar fields"), &test_to_json_scalar_fields},
	{FN("quaternion fields"), &test_to_json_quaternions},
	{FN("camera angles arrays"), &test_to_json_cam_angles_arrays},
	{FN("zero camera angles produces empty arrays"),
	 &test_to_json_zero_cam_angles},
	CU_TEST_INFO_NULL,
};
