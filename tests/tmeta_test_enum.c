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


static void test_gain_mode_from_str(void)
{
	CU_ASSERT_EQUAL(tmeta_thermal_gain_mode_from_str("FLIR_LOW_GAIN"),
			TMETA_THERMAL_GAIN_MODE_FLIR_LOW_GAIN);
	CU_ASSERT_EQUAL(tmeta_thermal_gain_mode_from_str("flir_low_gain"),
			TMETA_THERMAL_GAIN_MODE_FLIR_LOW_GAIN);
	CU_ASSERT_EQUAL(tmeta_thermal_gain_mode_from_str("Flir_Low_Gain"),
			TMETA_THERMAL_GAIN_MODE_FLIR_LOW_GAIN);
	CU_ASSERT_EQUAL(tmeta_thermal_gain_mode_from_str("FLIR_HIGH_GAIN"),
			TMETA_THERMAL_GAIN_MODE_FLIR_HIGH_GAIN);
	CU_ASSERT_EQUAL(tmeta_thermal_gain_mode_from_str("flir_high_gain"),
			TMETA_THERMAL_GAIN_MODE_FLIR_HIGH_GAIN);
}


static void test_gain_mode_from_str_unknown(void)
{
	CU_ASSERT_EQUAL(tmeta_thermal_gain_mode_from_str("bogus"),
			TMETA_THERMAL_GAIN_MODE_FLIR_LOW_GAIN);
	CU_ASSERT_EQUAL(tmeta_thermal_gain_mode_from_str(""),
			TMETA_THERMAL_GAIN_MODE_FLIR_LOW_GAIN);
}


static void test_gain_mode_from_str_null(void)
{
	CU_ASSERT_EQUAL(tmeta_thermal_gain_mode_from_str(NULL),
			TMETA_THERMAL_GAIN_MODE_FLIR_LOW_GAIN);
}


static void test_gain_mode_to_str(void)
{
	CU_ASSERT_STRING_EQUAL(tmeta_thermal_gain_mode_to_str(
				       TMETA_THERMAL_GAIN_MODE_FLIR_LOW_GAIN),
			       "FLIR_LOW_GAIN");
	CU_ASSERT_STRING_EQUAL(tmeta_thermal_gain_mode_to_str(
				       TMETA_THERMAL_GAIN_MODE_FLIR_HIGH_GAIN),
			       "FLIR_HIGH_GAIN");
	CU_ASSERT_STRING_EQUAL(tmeta_thermal_gain_mode_to_str(
				       (enum tmeta_thermal_gain_mode)99),
			       "UNKNOWN");
}


static void test_frame_state_from_str(void)
{
	CU_ASSERT_EQUAL(tmeta_thermal_frame_state_from_str("VALID"),
			TMETA_THERMAL_FRAME_STATE_VALID);
	CU_ASSERT_EQUAL(tmeta_thermal_frame_state_from_str("valid"),
			TMETA_THERMAL_FRAME_STATE_VALID);
	CU_ASSERT_EQUAL(tmeta_thermal_frame_state_from_str("SHUTTER_DESIRED"),
			TMETA_THERMAL_FRAME_STATE_SHUTTER_DESIRED);
	CU_ASSERT_EQUAL(
		tmeta_thermal_frame_state_from_str("Shutter_In_Progress"),
		TMETA_THERMAL_FRAME_STATE_SHUTTER_IN_PROGRESS);
	CU_ASSERT_EQUAL(tmeta_thermal_frame_state_from_str("UNEXPECTED"),
			TMETA_THERMAL_FRAME_STATE_UNEXPECTED);
}


static void test_frame_state_from_str_unknown(void)
{
	CU_ASSERT_EQUAL(tmeta_thermal_frame_state_from_str("bogus"),
			TMETA_THERMAL_FRAME_STATE_UNEXPECTED);
}


static void test_frame_state_from_str_null(void)
{
	CU_ASSERT_EQUAL(tmeta_thermal_frame_state_from_str(NULL),
			TMETA_THERMAL_FRAME_STATE_UNEXPECTED);
}


static void test_frame_state_to_str(void)
{
	CU_ASSERT_STRING_EQUAL(tmeta_thermal_frame_state_to_str(
				       TMETA_THERMAL_FRAME_STATE_VALID),
			       "VALID");
	CU_ASSERT_STRING_EQUAL(
		tmeta_thermal_frame_state_to_str(
			TMETA_THERMAL_FRAME_STATE_SHUTTER_DESIRED),
		"SHUTTER_DESIRED");
	CU_ASSERT_STRING_EQUAL(
		tmeta_thermal_frame_state_to_str(
			TMETA_THERMAL_FRAME_STATE_SHUTTER_IN_PROGRESS),
		"SHUTTER_IN_PROGRESS");
	CU_ASSERT_STRING_EQUAL(tmeta_thermal_frame_state_to_str(
				       TMETA_THERMAL_FRAME_STATE_UNEXPECTED),
			       "UNEXPECTED");
	CU_ASSERT_STRING_EQUAL(tmeta_thermal_frame_state_to_str(
				       (enum tmeta_thermal_frame_state)99),
			       "UNKNOWN");
}


static void test_version_macros(void)
{
	uint32_t v = (2u << 16) | 3u;

	CU_ASSERT_EQUAL(TMETA_GET_MAJOR_VERSION(TMETA_VERSION),
			(uint32_t)TMETA_MAJOR_VERSION);
	CU_ASSERT_EQUAL(TMETA_GET_MINOR_VERSION(TMETA_VERSION),
			(uint32_t)TMETA_MINOR_VERSION);
	CU_ASSERT_EQUAL(TMETA_GET_MAJOR_VERSION(v), 2u);
	CU_ASSERT_EQUAL(TMETA_GET_MINOR_VERSION(v), 3u);
}


static void test_version_macros_compound_expression_caveat(void)
{
	/* TMETA_GET_MAJOR_VERSION/TMETA_GET_MINOR_VERSION don't
	 * parenthesize their 'version' argument, so passing a compound
	 * expression directly (instead of a single pre-computed variable,
	 * which is how every real caller in this library uses them)
	 * silently produces the wrong result due to operator precedence.
	 * This documents the current, unfixed behavior; it is not a
	 * contract to preserve. */
	uint32_t major = 5, minor = 7;

	CU_ASSERT_NOT_EQUAL(TMETA_GET_MAJOR_VERSION(major << 16 | minor),
			    major);
}


CU_TestInfo s_enum_tests[] = {
	{FN("gain mode from string"), &test_gain_mode_from_str},
	{FN("gain mode from unknown string"), &test_gain_mode_from_str_unknown},
	{FN("gain mode from null string"), &test_gain_mode_from_str_null},
	{FN("gain mode to string"), &test_gain_mode_to_str},
	{FN("frame state from string"), &test_frame_state_from_str},
	{FN("frame state from unknown string"),
	 &test_frame_state_from_str_unknown},
	{FN("frame state from null string"), &test_frame_state_from_str_null},
	{FN("frame state to string"), &test_frame_state_to_str},
	{FN("version major/minor macros"), &test_version_macros},
	{FN("version macros compound expression caveat"),
	 &test_version_macros_compound_expression_caveat},
	CU_TEST_INFO_NULL,
};
