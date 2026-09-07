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

#define MIN_SEI_SIZE (TMETA_SEI_UUID_SIZE + TMETA_VERSION_SIZE)


static void put_uuid(uint8_t *buf,
		     uint32_t uuid0,
		     uint32_t uuid1,
		     uint32_t uuid2,
		     uint32_t uuid3)
{
	put_be32(buf + 0, uuid0);
	put_be32(buf + 4, uuid1);
	put_be32(buf + 8, uuid2);
	put_be32(buf + 12, uuid3);
}


static void test_sei_valid_uuid_min_size(void)
{
	uint8_t buf[MIN_SEI_SIZE];

	put_uuid(buf,
		 TMETA_USER_DATA_SEI_UUID_0,
		 TMETA_USER_DATA_SEI_UUID_1,
		 TMETA_USER_DATA_SEI_UUID_2,
		 TMETA_USER_DATA_SEI_UUID_3);
	put_be32(buf + 16, TMETA_VERSION);

	CU_ASSERT_TRUE(
		tmeta_is_thermal_metadata_user_data_sei(buf, sizeof(buf)));
}


static void test_sei_valid_uuid_larger_buffer(void)
{
	uint8_t buf[MIN_SEI_SIZE + 80];

	memset(buf, 0, sizeof(buf));
	put_uuid(buf,
		 TMETA_USER_DATA_SEI_UUID_0,
		 TMETA_USER_DATA_SEI_UUID_1,
		 TMETA_USER_DATA_SEI_UUID_2,
		 TMETA_USER_DATA_SEI_UUID_3);
	put_be32(buf + 16, TMETA_VERSION);

	CU_ASSERT_TRUE(
		tmeta_is_thermal_metadata_user_data_sei(buf, sizeof(buf)));
}


static void test_sei_wrong_uuid(void)
{
	uint8_t buf[MIN_SEI_SIZE];

	put_uuid(buf,
		 TMETA_USER_DATA_SEI_UUID_0,
		 TMETA_USER_DATA_SEI_UUID_1,
		 TMETA_USER_DATA_SEI_UUID_2,
		 0xdeadbeef);
	put_be32(buf + 16, TMETA_VERSION);

	CU_ASSERT_FALSE(
		tmeta_is_thermal_metadata_user_data_sei(buf, sizeof(buf)));
}


static void test_sei_too_short(void)
{
	uint8_t buf[MIN_SEI_SIZE];

	put_uuid(buf,
		 TMETA_USER_DATA_SEI_UUID_0,
		 TMETA_USER_DATA_SEI_UUID_1,
		 TMETA_USER_DATA_SEI_UUID_2,
		 TMETA_USER_DATA_SEI_UUID_3);
	put_be32(buf + 16, TMETA_VERSION);

	CU_ASSERT_FALSE(
		tmeta_is_thermal_metadata_user_data_sei(buf, sizeof(buf) - 1));
}


static void test_sei_null_buf(void)
{
	CU_ASSERT_FALSE(tmeta_is_thermal_metadata_user_data_sei(NULL, 100));
}


static void test_sei_zero_size(void)
{
	uint8_t buf[MIN_SEI_SIZE] = {0};

	CU_ASSERT_FALSE(tmeta_is_thermal_metadata_user_data_sei(buf, 0));
}


CU_TestInfo s_sei_tests[] = {
	{FN("valid uuid at minimum size"), &test_sei_valid_uuid_min_size},
	{FN("valid uuid with larger buffer"),
	 &test_sei_valid_uuid_larger_buffer},
	{FN("wrong uuid"), &test_sei_wrong_uuid},
	{FN("buffer one byte too short"), &test_sei_too_short},
	{FN("null buffer"), &test_sei_null_buf},
	{FN("zero size"), &test_sei_zero_size},
	CU_TEST_INFO_NULL,
};
