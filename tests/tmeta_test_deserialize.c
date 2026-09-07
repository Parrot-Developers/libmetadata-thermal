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

#define V0_1_END                                                               \
	(TMETA_SEI_UUID_SIZE + TMETA_VERSION_SIZE + TMETA_V0_1_HEADER_SIZE)
#define CAM_ANGLES_END(_count) (V0_1_END + 24 * (_count))
#define JPEG_END(_count, _jsize) (CAM_ANGLES_END(_count) + (_jsize))
#define V0_2_END(_count, _jsize)                                               \
	(JPEG_END(_count, _jsize) + TMETA_V0_2_DATA_SIZE)
#define V0_3_END(_count, _jsize)                                               \
	(V0_2_END(_count, _jsize) + TMETA_V0_3_DATA_SIZE)
#define V0_4_END(_count, _jsize)                                               \
	(V0_3_END(_count, _jsize) + TMETA_V0_4_DATA_SIZE)

#define TEST_BUF_SIZE 4096

#define SENTINEL_BYTE 0xAA


size_t build_wire_buffer(uint8_t *buf,
			 uint32_t version,
			 uint32_t cam_angles_count,
			 uint32_t jpeg_data_size,
			 const uint8_t *jpeg_data)
{
	uint8_t *p = buf;
	double d;
	float f;
	uint32_t i;

	put_be32(p, TMETA_USER_DATA_SEI_UUID_0);
	p += 4;
	put_be32(p, TMETA_USER_DATA_SEI_UUID_1);
	p += 4;
	put_be32(p, TMETA_USER_DATA_SEI_UUID_2);
	p += 4;
	put_be32(p, TMETA_USER_DATA_SEI_UUID_3);
	p += 4;

	put_be32(p, version);
	p += 4;

	put_be32(p, (uint32_t)TMETA_THERMAL_GAIN_MODE_FLIR_HIGH_GAIN);
	p += 4;

	d = 1.5;
	memcpy(p, &d, sizeof(d));
	p += sizeof(d);
	d = 2.5;
	memcpy(p, &d, sizeof(d));
	p += sizeof(d);
	d = 3.5;
	memcpy(p, &d, sizeof(d));
	p += sizeof(d);
	d = 4.5;
	memcpy(p, &d, sizeof(d));
	p += sizeof(d);
	d = 5.5;
	memcpy(p, &d, sizeof(d));
	p += sizeof(d);
	d = 6.5;
	memcpy(p, &d, sizeof(d));
	p += sizeof(d);
	d = 7.5;
	memcpy(p, &d, sizeof(d));
	p += sizeof(d);
	d = 8.5;
	memcpy(p, &d, sizeof(d));
	p += sizeof(d);

	put_be32(p, jpeg_data_size);
	p += 4;
	put_be32(p, 100);
	p += 4;
	put_be32(p, 60000);
	p += 4;

	f = 0.1f;
	memcpy(p, &f, sizeof(f));
	p += sizeof(f);
	f = 0.2f;
	memcpy(p, &f, sizeof(f));
	p += sizeof(f);
	f = 0.3f;
	memcpy(p, &f, sizeof(f));
	p += sizeof(f);
	f = 0.9f;
	memcpy(p, &f, sizeof(f));
	p += sizeof(f);

	put_be32(p, cam_angles_count);
	p += 4;

	for (i = 0; i < cam_angles_count; i++) {
		f = 0.01f * (float)i;
		memcpy(p, &f, sizeof(f));
		p += sizeof(f);
		f = 0.02f * (float)i;
		memcpy(p, &f, sizeof(f));
		p += sizeof(f);
		f = 0.03f * (float)i;
		memcpy(p, &f, sizeof(f));
		p += sizeof(f);
		f = 1.0f;
		memcpy(p, &f, sizeof(f));
		p += sizeof(f);
	}
	for (i = 0; i < cam_angles_count; i++) {
		put_be64(p, (uint64_t)(1000 + i));
		p += 8;
	}

	if (jpeg_data_size > 0) {
		memcpy(p, jpeg_data, jpeg_data_size);
		p += jpeg_data_size;
	}

	/* v0.2 */
	put_be32(p, (uint32_t)TMETA_THERMAL_FRAME_STATE_VALID);
	p += 4;

	/* v0.3 */
	d = 313.15;
	memcpy(p, &d, sizeof(d));
	p += sizeof(d);
	d = 300.0;
	memcpy(p, &d, sizeof(d));
	p += sizeof(d);
	d = 295.5;
	memcpy(p, &d, sizeof(d));
	p += sizeof(d);

	/* v0.4 */
	f = 0.01f;
	memcpy(p, &f, sizeof(f));
	p += sizeof(f);
	f = 0.02f;
	memcpy(p, &f, sizeof(f));
	p += sizeof(f);
	f = 0.03f;
	memcpy(p, &f, sizeof(f));
	p += sizeof(f);
	f = 0.99f;
	memcpy(p, &f, sizeof(f));
	p += sizeof(f);

	return (size_t)(p - buf);
}


static bool is_sentinel(const void *data, size_t size)
{
	const uint8_t *p = data;
	size_t i;

	for (i = 0; i < size; i++) {
		if (p[i] != SENTINEL_BYTE)
			return false;
	}
	return true;
}


static void test_deserialize_null_buf(void)
{
	struct tmeta_data meta;

	CU_ASSERT_EQUAL(tmeta_deserialize_thermal_metadata_user_data_sei(
				NULL, 100, &meta),
			-EINVAL);
}


static void test_deserialize_null_meta(void)
{
	uint8_t buf[TEST_BUF_SIZE];
	size_t len = build_wire_buffer(buf, TMETA_VERSION, 0, 0, NULL);

	CU_ASSERT_EQUAL(tmeta_deserialize_thermal_metadata_user_data_sei(
				buf, len, NULL),
			-EINVAL);
}


static void test_deserialize_wrong_uuid(void)
{
	uint8_t buf[TEST_BUF_SIZE];
	struct tmeta_data meta;
	size_t len = build_wire_buffer(buf, TMETA_VERSION, 0, 0, NULL);

	put_be32(buf + 12, 0xdeadbeef);

	CU_ASSERT_EQUAL(tmeta_deserialize_thermal_metadata_user_data_sei(
				buf, len, &meta),
			-ENOENT);
}


static void test_deserialize_too_short_for_uuid_version(void)
{
	uint8_t buf[TEST_BUF_SIZE];
	struct tmeta_data meta;

	build_wire_buffer(buf, TMETA_VERSION, 0, 0, NULL);

	CU_ASSERT_EQUAL(tmeta_deserialize_thermal_metadata_user_data_sei(
				buf,
				TMETA_SEI_UUID_SIZE + TMETA_VERSION_SIZE - 1,
				&meta),
			-ENOENT);
}


static void test_deserialize_major_version_too_high(void)
{
	uint8_t buf[TEST_BUF_SIZE];
	struct tmeta_data meta;
	uint32_t version = (1 << 16) | 0;
	size_t len = build_wire_buffer(buf, version, 0, 0, NULL);

	CU_ASSERT_EQUAL(tmeta_deserialize_thermal_metadata_user_data_sei(
				buf, len, &meta),
			-1);
}


static void test_deserialize_truncated_v0_1_header(void)
{
	uint8_t buf[TEST_BUF_SIZE];
	struct tmeta_data meta;
	uint32_t version = TMETA_MAJOR_VERSION << 16 | 1;

	build_wire_buffer(buf, version, 0, 0, NULL);

	CU_ASSERT_EQUAL(tmeta_deserialize_thermal_metadata_user_data_sei(
				buf, V0_1_END - 1, &meta),
			-1);
}


static void test_deserialize_truncated_cam_angles(void)
{
	uint8_t buf[TEST_BUF_SIZE];
	struct tmeta_data meta;
	uint32_t version = TMETA_MAJOR_VERSION << 16 | 1;

	build_wire_buffer(buf, version, 2, 0, NULL);

	CU_ASSERT_EQUAL(tmeta_deserialize_thermal_metadata_user_data_sei(
				buf, CAM_ANGLES_END(2) - 1, &meta),
			-1);
}


static void test_deserialize_truncated_jpeg_data(void)
{
	uint8_t buf[TEST_BUF_SIZE];
	struct tmeta_data meta;
	uint32_t version = TMETA_MAJOR_VERSION << 16 | 1;
	uint8_t jpeg[5] = {1, 2, 3, 4, 5};

	build_wire_buffer(buf, version, 0, sizeof(jpeg), jpeg);

	CU_ASSERT_EQUAL(tmeta_deserialize_thermal_metadata_user_data_sei(
				buf, JPEG_END(0, sizeof(jpeg)) - 1, &meta),
			-1);
}


static void test_deserialize_v0_1_minimal_success(void)
{
	uint8_t buf[TEST_BUF_SIZE];
	struct tmeta_data meta;
	uint32_t version = TMETA_MAJOR_VERSION << 16 | 1;

	build_wire_buffer(buf, version, 0, 0, NULL);
	memset(&meta, SENTINEL_BYTE, sizeof(meta));

	CU_ASSERT_EQUAL(tmeta_deserialize_thermal_metadata_user_data_sei(
				buf, JPEG_END(0, 0), &meta),
			0);

	CU_ASSERT_EQUAL(meta.version, version);
	CU_ASSERT_EQUAL(meta.gain_mode, TMETA_THERMAL_GAIN_MODE_FLIR_HIGH_GAIN);
	CU_ASSERT_EQUAL(meta.calib_r, 1.5);
	CU_ASSERT_EQUAL(meta.value_min, 100);
	CU_ASSERT_EQUAL(meta.value_max, 60000);
	CU_ASSERT_EQUAL(meta.cam_angles_count, 0);
	CU_ASSERT_EQUAL(meta.jpeg_data_size, 0);

	/* Fields added in v0.2+ must not have been touched */
	CU_ASSERT_TRUE(
		is_sentinel(&meta.frame_state, sizeof(meta.frame_state)));
	CU_ASSERT_TRUE(is_sentinel(&meta.fpa_temp, sizeof(meta.fpa_temp)));
	CU_ASSERT_TRUE(is_sentinel(&meta.thermal_to_visible_quat,
				   sizeof(meta.thermal_to_visible_quat)));
}


static void test_deserialize_v0_2_truncated(void)
{
	uint8_t buf[TEST_BUF_SIZE];
	struct tmeta_data meta;
	uint32_t version = TMETA_MAJOR_VERSION << 16 | 2;

	build_wire_buffer(buf, version, 0, 0, NULL);

	CU_ASSERT_EQUAL(tmeta_deserialize_thermal_metadata_user_data_sei(
				buf, V0_2_END(0, 0) - 1, &meta),
			-1);
}


static void test_deserialize_v0_2_adds_frame_state(void)
{
	uint8_t buf[TEST_BUF_SIZE];
	struct tmeta_data meta;
	uint32_t version = TMETA_MAJOR_VERSION << 16 | 2;

	build_wire_buffer(buf, version, 0, 0, NULL);
	memset(&meta, SENTINEL_BYTE, sizeof(meta));

	CU_ASSERT_EQUAL(tmeta_deserialize_thermal_metadata_user_data_sei(
				buf, V0_2_END(0, 0), &meta),
			0);

	CU_ASSERT_EQUAL(meta.frame_state, TMETA_THERMAL_FRAME_STATE_VALID);
	CU_ASSERT_TRUE(is_sentinel(&meta.fpa_temp, sizeof(meta.fpa_temp)));
	CU_ASSERT_TRUE(is_sentinel(&meta.thermal_to_visible_quat,
				   sizeof(meta.thermal_to_visible_quat)));
}


static void test_deserialize_v0_3_truncated(void)
{
	uint8_t buf[TEST_BUF_SIZE];
	struct tmeta_data meta;
	uint32_t version = TMETA_MAJOR_VERSION << 16 | 3;

	build_wire_buffer(buf, version, 0, 0, NULL);

	CU_ASSERT_EQUAL(tmeta_deserialize_thermal_metadata_user_data_sei(
				buf, V0_3_END(0, 0) - 1, &meta),
			-1);
}


static void test_deserialize_v0_3_adds_temperatures(void)
{
	uint8_t buf[TEST_BUF_SIZE];
	struct tmeta_data meta;
	uint32_t version = TMETA_MAJOR_VERSION << 16 | 3;

	build_wire_buffer(buf, version, 0, 0, NULL);
	memset(&meta, SENTINEL_BYTE, sizeof(meta));

	CU_ASSERT_EQUAL(tmeta_deserialize_thermal_metadata_user_data_sei(
				buf, V0_3_END(0, 0), &meta),
			0);

	CU_ASSERT_EQUAL(meta.fpa_temp, 313.15);
	CU_ASSERT_EQUAL(meta.housing_temp, 300.0);
	CU_ASSERT_EQUAL(meta.window_reflection, 295.5);
	CU_ASSERT_TRUE(is_sentinel(&meta.thermal_to_visible_quat,
				   sizeof(meta.thermal_to_visible_quat)));
}


static void test_deserialize_v0_4_truncated(void)
{
	uint8_t buf[TEST_BUF_SIZE];
	struct tmeta_data meta;
	uint32_t version = TMETA_MAJOR_VERSION << 16 | 4;

	build_wire_buffer(buf, version, 0, 0, NULL);

	CU_ASSERT_EQUAL(tmeta_deserialize_thermal_metadata_user_data_sei(
				buf, V0_4_END(0, 0) - 1, &meta),
			-1);
}


static void test_deserialize_v0_4_full_success(void)
{
	uint8_t buf[TEST_BUF_SIZE];
	struct tmeta_data meta;
	uint32_t version = TMETA_MAJOR_VERSION << 16 | 4;
	size_t len = build_wire_buffer(buf, version, 0, 0, NULL);

	memset(&meta, SENTINEL_BYTE, sizeof(meta));

	CU_ASSERT_EQUAL(tmeta_deserialize_thermal_metadata_user_data_sei(
				buf, len, &meta),
			0);

	CU_ASSERT_EQUAL(meta.thermal_to_visible_quat[0], 0.01f);
	CU_ASSERT_EQUAL(meta.thermal_to_visible_quat[3], 0.99f);
}


static void test_deserialize_minor_version_beyond_known_still_parses(void)
{
	uint8_t buf[TEST_BUF_SIZE];
	struct tmeta_data meta;
	uint32_t version = TMETA_MAJOR_VERSION << 16 | 99;
	size_t len = build_wire_buffer(buf, version, 0, 0, NULL);

	CU_ASSERT_EQUAL(tmeta_deserialize_thermal_metadata_user_data_sei(
				buf, len, &meta),
			0);

	CU_ASSERT_EQUAL(meta.thermal_to_visible_quat[0], 0.01f);
}


static void test_deserialize_jpeg_data_aliases_input_buffer(void)
{
	uint8_t buf[TEST_BUF_SIZE];
	struct tmeta_data meta;
	uint8_t jpeg[4] = {0x11, 0x22, 0x33, 0x44};
	size_t len =
		build_wire_buffer(buf, TMETA_VERSION, 0, sizeof(jpeg), jpeg);

	CU_ASSERT_EQUAL(tmeta_deserialize_thermal_metadata_user_data_sei(
				buf, len, &meta),
			0);

	CU_ASSERT_PTR_EQUAL(meta.jpeg_data, buf + CAM_ANGLES_END(0));
	CU_ASSERT_EQUAL(memcmp(meta.jpeg_data, jpeg, sizeof(jpeg)), 0);

	/* jpeg_data is not copied: mutating the input buffer after
	 * deserialize must be visible through meta.jpeg_data */
	buf[CAM_ANGLES_END(0)] = 0xff;
	CU_ASSERT_EQUAL(*(uint8_t *)meta.jpeg_data, 0xff);
}


static void test_deserialize_cam_angles_count_exceeds_max(void)
{
	uint8_t buf[TEST_BUF_SIZE];
	struct tmeta_data meta;
	size_t len = build_wire_buffer(
		buf, TMETA_VERSION, TMETA_CAMANGLES_MAXCOUNT + 1, 0, NULL);

	CU_ASSERT_EQUAL(tmeta_deserialize_thermal_metadata_user_data_sei(
				buf, len, &meta),
			-1);
}


CU_TestInfo s_deserialize_tests[] = {
	{FN("null buffer"), &test_deserialize_null_buf},
	{FN("null meta"), &test_deserialize_null_meta},
	{FN("wrong uuid"), &test_deserialize_wrong_uuid},
	{FN("too short for uuid+version"),
	 &test_deserialize_too_short_for_uuid_version},
	{FN("major version too high"),
	 &test_deserialize_major_version_too_high},
	{FN("truncated v0.1 header"), &test_deserialize_truncated_v0_1_header},
	{FN("truncated camera angles"), &test_deserialize_truncated_cam_angles},
	{FN("truncated jpeg data"), &test_deserialize_truncated_jpeg_data},
	{FN("v0.1 minimal buffer succeeds"),
	 &test_deserialize_v0_1_minimal_success},
	{FN("v0.2 truncated"), &test_deserialize_v0_2_truncated},
	{FN("v0.2 adds frame state"), &test_deserialize_v0_2_adds_frame_state},
	{FN("v0.3 truncated"), &test_deserialize_v0_3_truncated},
	{FN("v0.3 adds temperatures"),
	 &test_deserialize_v0_3_adds_temperatures},
	{FN("v0.4 truncated"), &test_deserialize_v0_4_truncated},
	{FN("v0.4 full buffer succeeds"), &test_deserialize_v0_4_full_success},
	{FN("minor version beyond known still parses as v0.4"),
	 &test_deserialize_minor_version_beyond_known_still_parses},
	{FN("jpeg data aliases input buffer"),
	 &test_deserialize_jpeg_data_aliases_input_buffer},
	{FN("cam_angles_count exceeding maximum is rejected"),
	 &test_deserialize_cam_angles_count_exceeds_max},
	CU_TEST_INFO_NULL,
};
