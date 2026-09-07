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

#ifndef _TMETA_TEST_H_
#define _TMETA_TEST_H_

#include <metadata-thermal/tmeta.h>

#include <CUnit/Automated.h>
#include <CUnit/Basic.h>
#include <CUnit/CUnit.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define ULOG_TAG tmeta_test
#include <ulog.h>


#define FN(_name) (char *)_name


extern CU_TestInfo s_sei_tests[];
extern CU_TestInfo s_serialize_tests[];
extern CU_TestInfo s_deserialize_tests[];
extern CU_TestInfo s_roundtrip_tests[];
extern CU_TestInfo s_enum_tests[];
extern CU_TestInfo s_json_tests[];


/* Big-endian byte writers, used to hand-build wire buffers independently
 * of the library's own htonl/htonll-based serialization, so that tests
 * actually validate the wire format rather than the library against
 * itself */
static inline void put_be32(uint8_t *buf, uint32_t val)
{
	buf[0] = (uint8_t)(val >> 24);
	buf[1] = (uint8_t)(val >> 16);
	buf[2] = (uint8_t)(val >> 8);
	buf[3] = (uint8_t)(val);
}


static inline void put_be64(uint8_t *buf, uint64_t val)
{
	buf[0] = (uint8_t)(val >> 56);
	buf[1] = (uint8_t)(val >> 48);
	buf[2] = (uint8_t)(val >> 40);
	buf[3] = (uint8_t)(val >> 32);
	buf[4] = (uint8_t)(val >> 24);
	buf[5] = (uint8_t)(val >> 16);
	buf[6] = (uint8_t)(val >> 8);
	buf[7] = (uint8_t)(val);
}


/* Hand-builds a complete thermal metadata user data SEI wire buffer
 * (UUID + version + v0.1 header + camera angles + JPEG data + v0.2 + v0.3
 * + v0.4 sections, unconditionally, regardless of the given version's
 * minor number) into buf, and returns the total number of bytes written.
 * buf must be at least
 * TMETA_SEI_UUID_SIZE + TMETA_VERSION_SIZE + TMETA_V0_1_HEADER_SIZE +
 * 16 * cam_angles_count + 8 * cam_angles_count + jpeg_data_size +
 * TMETA_V0_2_DATA_SIZE + TMETA_V0_3_DATA_SIZE + TMETA_V0_4_DATA_SIZE bytes.
 * Truncating the resulting buffer (i.e. passing a smaller buf_size than
 * what this function returns to the function under test) is how boundary
 * / truncated-buffer tests are constructed; setting version's minor number
 * lower than 4 is how "declares an older format" tests are constructed
 * (the deserializer stops early based on the declared version, regardless
 * of how many extra bytes physically follow in the buffer). */
size_t build_wire_buffer(uint8_t *buf,
			 uint32_t version,
			 uint32_t cam_angles_count,
			 uint32_t jpeg_data_size,
			 const uint8_t *jpeg_data);


/* Fills meta with deterministic, distinct values for every field so that
 * a serialize/deserialize round trip can be verified field-by-field.
 * meta is fully overwritten (memset to 0 first). jpeg_data must remain
 * valid for as long as meta is used to serialize. */
void fill_tmeta_data(struct tmeta_data *meta,
		     uint32_t cam_angles_count,
		     uint32_t jpeg_data_size,
		     void *jpeg_data);


/* Compares every field of m1 and m2 except 'version' (the caller must
 * check that separately, since tmeta_serialize_thermal_metadata_user_data_sei
 * always forces the output version to TMETA_VERSION regardless of the
 * input struct's 'version' field). jpeg_data is compared by content
 * (memcmp over jpeg_data_size bytes), not by pointer identity, since
 * tmeta_deserialize_thermal_metadata_user_data_sei() aliases jpeg_data
 * into the input wire buffer rather than copying it. */
void compare_tmeta_data(const struct tmeta_data *m1,
			const struct tmeta_data *m2);


#endif /* !_TMETA_TEST_H_ */
