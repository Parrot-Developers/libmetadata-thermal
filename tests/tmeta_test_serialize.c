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


static uint32_t get_be32(const uint8_t *buf)
{
	return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
	       ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
}


static void test_serialize_null_meta(void)
{
	uint8_t buf[256];

	CU_ASSERT_EQUAL(tmeta_serialize_thermal_metadata_user_data_sei(
				NULL, buf, sizeof(buf), NULL),
			-EINVAL);
}


static void test_serialize_null_buf(void)
{
	struct tmeta_data meta;
	uint8_t dummy = 0;

	fill_tmeta_data(&meta, 0, 0, &dummy);

	CU_ASSERT_EQUAL(tmeta_serialize_thermal_metadata_user_data_sei(
				&meta, NULL, 1000, NULL),
			-EINVAL);
}


static void test_serialize_buffer_too_small(void)
{
	struct tmeta_data meta;
	uint8_t dummy = 0;
	uint8_t *buf;
	size_t bufsize;
	int res;

	fill_tmeta_data(&meta, 0, 0, &dummy);
	bufsize = TMETA_BUF_SIZE(&meta);
	buf = malloc(bufsize);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);

	res = tmeta_serialize_thermal_metadata_user_data_sei(
		&meta, buf, bufsize - 1, NULL);
	CU_ASSERT_EQUAL(res, -ENOBUFS);

	free(buf);
}


static void test_serialize_buffer_exact_size_succeeds(void)
{
	struct tmeta_data meta;
	uint8_t dummy = 0;
	uint8_t *buf;
	size_t bufsize, written = 0;
	int res;

	fill_tmeta_data(&meta, 0, 0, &dummy);
	bufsize = TMETA_BUF_SIZE(&meta);
	buf = malloc(bufsize);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);

	res = tmeta_serialize_thermal_metadata_user_data_sei(
		&meta, buf, bufsize, &written);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(written, bufsize);

	free(buf);
}


static void test_serialize_size_output_optional(void)
{
	struct tmeta_data meta;
	uint8_t dummy = 0;
	uint8_t *buf;
	size_t bufsize;
	int res;

	fill_tmeta_data(&meta, 0, 0, &dummy);
	bufsize = TMETA_BUF_SIZE(&meta);
	buf = malloc(bufsize);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);

	res = tmeta_serialize_thermal_metadata_user_data_sei(
		&meta, buf, bufsize, NULL);
	CU_ASSERT_EQUAL(res, 0);

	free(buf);
}


static void test_serialize_writes_correct_uuid_and_forces_version(void)
{
	struct tmeta_data meta;
	uint8_t dummy = 0;
	uint8_t *buf;
	size_t bufsize, written = 0;
	int res;

	fill_tmeta_data(&meta, 0, 0, &dummy);
	meta.version = 0xdeadbeef; /* bogus input, must not appear in output */
	bufsize = TMETA_BUF_SIZE(&meta);
	buf = malloc(bufsize);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);

	res = tmeta_serialize_thermal_metadata_user_data_sei(
		&meta, buf, bufsize, &written);
	CU_ASSERT_EQUAL(res, 0);

	CU_ASSERT_EQUAL(get_be32(buf + 0), TMETA_USER_DATA_SEI_UUID_0);
	CU_ASSERT_EQUAL(get_be32(buf + 4), TMETA_USER_DATA_SEI_UUID_1);
	CU_ASSERT_EQUAL(get_be32(buf + 8), TMETA_USER_DATA_SEI_UUID_2);
	CU_ASSERT_EQUAL(get_be32(buf + 12), TMETA_USER_DATA_SEI_UUID_3);
	CU_ASSERT_EQUAL(get_be32(buf + 16), (uint32_t)TMETA_VERSION);

	free(buf);
}


static void test_TMETA_BUF_SIZE_macro(void)
{
	struct tmeta_data meta;

	memset(&meta, 0, sizeof(meta));

	meta.cam_angles_count = 0;
	meta.jpeg_data_size = 0;
	CU_ASSERT_EQUAL(TMETA_BUF_SIZE(&meta), (size_t)164);

	meta.cam_angles_count = 2;
	meta.jpeg_data_size = 0;
	CU_ASSERT_EQUAL(TMETA_BUF_SIZE(&meta), (size_t)212);

	meta.cam_angles_count = 2;
	meta.jpeg_data_size = 10;
	CU_ASSERT_EQUAL(TMETA_BUF_SIZE(&meta), (size_t)222);
}


static void test_serialize_cam_angles_count_exceeds_max(void)
{
	struct tmeta_data meta;
	uint8_t dummy = 0;
	uint8_t buf[8192];

	fill_tmeta_data(&meta, 0, 0, &dummy);
	meta.cam_angles_count = TMETA_CAMANGLES_MAXCOUNT + 1;

	CU_ASSERT_EQUAL(tmeta_serialize_thermal_metadata_user_data_sei(
				&meta, buf, sizeof(buf), NULL),
			-EINVAL);
}


static void test_serialize_cam_angles_count_exactly_max(void)
{
	struct tmeta_data meta;
	uint8_t dummy = 0;
	uint8_t *buf;
	size_t bufsize, written = 0;
	int res;

	fill_tmeta_data(&meta, TMETA_CAMANGLES_MAXCOUNT, 0, &dummy);
	bufsize = TMETA_BUF_SIZE(&meta);
	buf = malloc(bufsize);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);

	res = tmeta_serialize_thermal_metadata_user_data_sei(
		&meta, buf, bufsize, &written);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(written, bufsize);

	free(buf);
}


static void test_serialize_null_jpeg_data_with_nonzero_size(void)
{
	struct tmeta_data meta;
	uint8_t buf[8192];

	fill_tmeta_data(&meta, 0, 10, NULL);

	CU_ASSERT_EQUAL(tmeta_serialize_thermal_metadata_user_data_sei(
				&meta, buf, sizeof(buf), NULL),
			-EINVAL);
}


CU_TestInfo s_serialize_tests[] = {
	{FN("null meta"), &test_serialize_null_meta},
	{FN("null buf"), &test_serialize_null_buf},
	{FN("buffer one byte too small"), &test_serialize_buffer_too_small},
	{FN("buffer exact size succeeds"),
	 &test_serialize_buffer_exact_size_succeeds},
	{FN("size output pointer is optional"),
	 &test_serialize_size_output_optional},
	{FN("writes correct uuid and forces current version"),
	 &test_serialize_writes_correct_uuid_and_forces_version},
	{FN("TMETA_BUF_SIZE macro arithmetic"), &test_TMETA_BUF_SIZE_macro},
	{FN("cam_angles_count exceeding maximum is rejected"),
	 &test_serialize_cam_angles_count_exceeds_max},
	{FN("cam_angles_count exactly at maximum succeeds"),
	 &test_serialize_cam_angles_count_exactly_max},
	{FN("null jpeg_data with nonzero jpeg_data_size is rejected"),
	 &test_serialize_null_jpeg_data_with_nonzero_size},
	CU_TEST_INFO_NULL,
};
