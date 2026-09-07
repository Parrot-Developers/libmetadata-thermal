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


void fill_tmeta_data(struct tmeta_data *meta,
		     uint32_t cam_angles_count,
		     uint32_t jpeg_data_size,
		     void *jpeg_data)
{
	uint32_t i;

	memset(meta, 0, sizeof(*meta));

	meta->version = TMETA_VERSION;
	meta->gain_mode = TMETA_THERMAL_GAIN_MODE_FLIR_HIGH_GAIN;
	meta->calib_r = 1.234;
	meta->calib_b = -5678.9;
	meta->calib_f = 12.3;
	meta->calib_o = 0.001;
	meta->calib_tau_win = 0.98;
	meta->calib_t_win = 300.5;
	meta->calib_t_bg = 290.1;
	meta->calib_emissivity = 0.95;
	meta->jpeg_data_size = jpeg_data_size;
	meta->jpeg_data = jpeg_data;
	meta->value_min = 1000;
	meta->value_max = 60000;
	meta->attitude_reference_quat[0] = 0.1f;
	meta->attitude_reference_quat[1] = 0.2f;
	meta->attitude_reference_quat[2] = 0.3f;
	meta->attitude_reference_quat[3] = 0.9f;

	meta->cam_angles_count = cam_angles_count;
	for (i = 0; i < cam_angles_count; i++) {
		meta->cam_angles[i * 4 + 0] = 0.01f * (float)i;
		meta->cam_angles[i * 4 + 1] = 0.02f * (float)i;
		meta->cam_angles[i * 4 + 2] = 0.03f * (float)i;
		meta->cam_angles[i * 4 + 3] = 1.0f;
		meta->cam_angles_timestamps[i] = 1000000ULL * (i + 1);
	}

	meta->frame_state = TMETA_THERMAL_FRAME_STATE_SHUTTER_IN_PROGRESS;
	meta->fpa_temp = 313.15;
	meta->housing_temp = 300.0;
	meta->window_reflection = 295.5;
	meta->thermal_to_visible_quat[0] = 0.01f;
	meta->thermal_to_visible_quat[1] = 0.02f;
	meta->thermal_to_visible_quat[2] = 0.03f;
	meta->thermal_to_visible_quat[3] = 0.99f;
}


void compare_tmeta_data(const struct tmeta_data *m1,
			const struct tmeta_data *m2)
{
	uint32_t i;
	uint32_t common_count = m1->cam_angles_count < m2->cam_angles_count
					? m1->cam_angles_count
					: m2->cam_angles_count;

	CU_ASSERT_EQUAL(m1->gain_mode, m2->gain_mode);
	CU_ASSERT_EQUAL(m1->calib_r, m2->calib_r);
	CU_ASSERT_EQUAL(m1->calib_b, m2->calib_b);
	CU_ASSERT_EQUAL(m1->calib_f, m2->calib_f);
	CU_ASSERT_EQUAL(m1->calib_o, m2->calib_o);
	CU_ASSERT_EQUAL(m1->calib_tau_win, m2->calib_tau_win);
	CU_ASSERT_EQUAL(m1->calib_t_win, m2->calib_t_win);
	CU_ASSERT_EQUAL(m1->calib_t_bg, m2->calib_t_bg);
	CU_ASSERT_EQUAL(m1->calib_emissivity, m2->calib_emissivity);
	CU_ASSERT_EQUAL(m1->jpeg_data_size, m2->jpeg_data_size);
	CU_ASSERT_EQUAL(m1->value_min, m2->value_min);
	CU_ASSERT_EQUAL(m1->value_max, m2->value_max);

	for (i = 0; i < 4; i++) {
		CU_ASSERT_EQUAL(m1->attitude_reference_quat[i],
				m2->attitude_reference_quat[i]);
	}

	CU_ASSERT_EQUAL(m1->cam_angles_count, m2->cam_angles_count);
	for (i = 0; i < common_count * 4; i++)
		CU_ASSERT_EQUAL(m1->cam_angles[i], m2->cam_angles[i]);
	for (i = 0; i < common_count; i++) {
		CU_ASSERT_EQUAL(m1->cam_angles_timestamps[i],
				m2->cam_angles_timestamps[i]);
	}

	if (m1->jpeg_data_size > 0 && m1->jpeg_data != NULL &&
	    m2->jpeg_data != NULL) {
		CU_ASSERT_EQUAL(memcmp(m1->jpeg_data,
				       m2->jpeg_data,
				       m1->jpeg_data_size),
				0);
	}

	CU_ASSERT_EQUAL(m1->frame_state, m2->frame_state);
	CU_ASSERT_EQUAL(m1->fpa_temp, m2->fpa_temp);
	CU_ASSERT_EQUAL(m1->housing_temp, m2->housing_temp);
	CU_ASSERT_EQUAL(m1->window_reflection, m2->window_reflection);
	for (i = 0; i < 4; i++) {
		CU_ASSERT_EQUAL(m1->thermal_to_visible_quat[i],
				m2->thermal_to_visible_quat[i]);
	}
}


static void test_roundtrip_zero_cam_angles_empty_jpeg(void)
{
	struct tmeta_data src, dst;
	uint8_t dummy = 0;
	uint8_t *buf;
	size_t bufsize, written = 0;
	int res;

	fill_tmeta_data(&src, 0, 0, &dummy);
	bufsize = TMETA_BUF_SIZE(&src);
	buf = malloc(bufsize);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);

	res = tmeta_serialize_thermal_metadata_user_data_sei(
		&src, buf, bufsize, &written);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(written, bufsize);

	memset(&dst, 0, sizeof(dst));
	res = tmeta_deserialize_thermal_metadata_user_data_sei(
		buf, written, &dst);
	CU_ASSERT_EQUAL(res, 0);

	compare_tmeta_data(&src, &dst);
	CU_ASSERT_EQUAL(dst.version, TMETA_VERSION);

	free(buf);
}


static void test_roundtrip_several_cam_angles_and_jpeg(void)
{
	struct tmeta_data src, dst;
	uint8_t jpeg[8] = {1, 2, 3, 4, 5, 6, 7, 8};
	uint8_t *buf;
	size_t bufsize, written = 0;
	int res;

	fill_tmeta_data(&src, 3, sizeof(jpeg), jpeg);
	bufsize = TMETA_BUF_SIZE(&src);
	buf = malloc(bufsize);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);

	res = tmeta_serialize_thermal_metadata_user_data_sei(
		&src, buf, bufsize, &written);
	CU_ASSERT_EQUAL(res, 0);

	memset(&dst, 0, sizeof(dst));
	res = tmeta_deserialize_thermal_metadata_user_data_sei(
		buf, written, &dst);
	CU_ASSERT_EQUAL(res, 0);

	compare_tmeta_data(&src, &dst);

	free(buf);
}


static void test_roundtrip_max_cam_angles(void)
{
	struct tmeta_data src, dst;
	uint8_t dummy = 0;
	uint8_t *buf;
	size_t bufsize, written = 0;
	int res;

	fill_tmeta_data(&src, TMETA_CAMANGLES_MAXCOUNT, 0, &dummy);
	bufsize = TMETA_BUF_SIZE(&src);
	buf = malloc(bufsize);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);

	res = tmeta_serialize_thermal_metadata_user_data_sei(
		&src, buf, bufsize, &written);
	CU_ASSERT_EQUAL(res, 0);

	memset(&dst, 0, sizeof(dst));
	res = tmeta_deserialize_thermal_metadata_user_data_sei(
		buf, written, &dst);
	CU_ASSERT_EQUAL(res, 0);

	compare_tmeta_data(&src, &dst);

	free(buf);
}


static void test_roundtrip_version_normalized(void)
{
	struct tmeta_data src, dst;
	uint8_t dummy = 0;
	uint8_t *buf;
	size_t bufsize, written = 0;
	int res;

	fill_tmeta_data(&src, 0, 0, &dummy);
	src.version = 0x12345678; /* bogus, must be ignored on output */
	bufsize = TMETA_BUF_SIZE(&src);
	buf = malloc(bufsize);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);

	res = tmeta_serialize_thermal_metadata_user_data_sei(
		&src, buf, bufsize, &written);
	CU_ASSERT_EQUAL(res, 0);

	memset(&dst, 0, sizeof(dst));
	res = tmeta_deserialize_thermal_metadata_user_data_sei(
		buf, written, &dst);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_EQUAL(dst.version, TMETA_VERSION);
	CU_ASSERT_NOT_EQUAL(dst.version, src.version);

	free(buf);
}


static void test_roundtrip_extreme_calibration_values(void)
{
	struct tmeta_data src, dst;
	uint8_t dummy = 0;
	uint8_t *buf;
	size_t bufsize, written = 0;
	int res;

	fill_tmeta_data(&src, 0, 0, &dummy);
	src.calib_r = -1.0e300;
	src.calib_b = 1.0e-300;
	src.calib_f = -0.0;
	src.calib_o = 123456789.123456;
	bufsize = TMETA_BUF_SIZE(&src);
	buf = malloc(bufsize);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);

	res = tmeta_serialize_thermal_metadata_user_data_sei(
		&src, buf, bufsize, &written);
	CU_ASSERT_EQUAL(res, 0);

	memset(&dst, 0, sizeof(dst));
	res = tmeta_deserialize_thermal_metadata_user_data_sei(
		buf, written, &dst);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_EQUAL(dst.calib_r, src.calib_r);
	CU_ASSERT_EQUAL(dst.calib_b, src.calib_b);
	CU_ASSERT_EQUAL(dst.calib_o, src.calib_o);

	free(buf);
}


CU_TestInfo s_roundtrip_tests[] = {
	{FN("zero cam angles, empty jpeg"),
	 &test_roundtrip_zero_cam_angles_empty_jpeg},
	{FN("several cam angles and jpeg payload"),
	 &test_roundtrip_several_cam_angles_and_jpeg},
	{FN("maximum cam angles count"), &test_roundtrip_max_cam_angles},
	{FN("input version is normalized to current version"),
	 &test_roundtrip_version_normalized},
	{FN("extreme calibration values"),
	 &test_roundtrip_extreme_calibration_values},
	CU_TEST_INFO_NULL,
};
