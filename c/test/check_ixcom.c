/*
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <check.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ixcom/decode.h>
#include <ixcom/messages.h>

#include "check_suites.h"

#define FLOAT_EPS 1e-6

void msg_header_equals(const XCOMHeader *msg_in, const XCOMHeader *msg_out) {
  ck_assert_uint_eq(msg_in->sync, msg_out->sync);
  ck_assert_uint_eq(msg_in->msg_id, msg_out->msg_id);
  ck_assert_uint_eq(msg_in->frame_counter, msg_out->frame_counter);
  ck_assert_uint_eq(msg_in->trigger_source, msg_out->trigger_source);
  ck_assert_uint_eq(msg_in->msg_len, msg_out->msg_len);
  ck_assert_uint_eq(msg_in->gps_week, msg_out->gps_week);
  ck_assert_uint_eq(msg_in->gps_time_sec, msg_out->gps_time_sec);
  ck_assert_uint_eq(msg_in->gps_time_usec, msg_out->gps_time_usec);
}

void msg_footer_equals(const XCOMFooter *msg_in, const XCOMFooter *msg_out) {
  ck_assert_uint_eq(msg_in->global_status.value, msg_out->global_status.value);
  ck_assert_uint_eq(msg_in->crc16, msg_out->crc16);
}

void msg_imuraw_equals(const XCOMmsg_IMURAW *msg_in,
                       const XCOMmsg_IMURAW *msg_out) {
  msg_header_equals(&msg_in->header, &msg_out->header);
  for (int i = 0; i < 3; i++) {
    ck_assert(fabs(msg_in->acc[i] - msg_out->acc[i]) < FLOAT_EPS);
    ck_assert(fabs(msg_in->omg[i] - msg_out->omg[i]) < FLOAT_EPS);
  }
  msg_footer_equals(&msg_in->footer, &msg_out->footer);
}

START_TEST(test_ixcom_imuraw) {}
END_TEST

Suite *ixcom_suite(void) {
  Suite *s = suite_create("ixcom");

  TCase *tc_ixcom = tcase_create("ixcom");
  tcase_add_test(tc_ixcom, test_ixcom_imuraw);
  suite_add_tcase(s, tc_ixcom);

  return s;
}
