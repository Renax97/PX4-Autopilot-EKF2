/****************************************************************************
 *
 *   Copyright (C) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <gtest/gtest.h>
#include <matrix/math.hpp>
#include "ObstacleMath.hpp"

using namespace matrix;

TEST(ObstacleMathTest, ProjectDistanceOnHorizontalPlane)
{
	// standard vehicle orientation inputs
	Quatf vehicle_pitch_up_45(Eulerf(0.0f, M_PI_4_F, 0.0f));
	Quatf vehicle_roll_right_45(Eulerf(M_PI_4_F, 0.0f, 0.0f));

	// GIVEN: a distance, sensor orientation, and quaternion representing the vehicle's orientation
	float distance = 1.0f;
	float sensor_orientation = 0; // radians (forward facing)

	// WHEN: we project the distance onto the horizontal plane
	ObstacleMath::project_distance_on_horizontal_plane(distance, sensor_orientation, vehicle_pitch_up_45);

	// THEN: the distance should be scaled correctly
	float expected_scale    = sqrtf(2) / 2;
	float expected_distance = 1.0f * expected_scale;

	EXPECT_NEAR(distance, expected_distance, 1e-5);

	// GIVEN: a distance, sensor orientation, and quaternion representing the vehicle's orientation
	distance = 1.0f;

	ObstacleMath::project_distance_on_horizontal_plane(distance, sensor_orientation, vehicle_roll_right_45);

	// THEN: the distance should be scaled correctly
	expected_scale     = 1.f;
	expected_distance  = 1.0f * expected_scale;

	EXPECT_NEAR(distance, expected_distance, 1e-5);

	// GIVEN: a distance, sensor orientation, and quaternion representing the vehicle's orientation
	distance = 1.0f;
	sensor_orientation = M_PI_2_F; // radians (right facing)

	ObstacleMath::project_distance_on_horizontal_plane(distance, sensor_orientation, vehicle_roll_right_45);

	// THEN: the distance should be scaled correctly
	expected_scale     = sqrtf(2) / 2;
	expected_distance  = 1.0f * expected_scale;

	EXPECT_NEAR(distance, expected_distance, 1e-5);

	// GIVEN: a distance, sensor orientation, and quaternion representing the vehicle's orientation
	distance = 1.0f;

	ObstacleMath::project_distance_on_horizontal_plane(distance, sensor_orientation, vehicle_pitch_up_45);

	// THEN: the distance should be scaled correctly
	expected_scale     = 1.f;
	expected_distance  = 1.0f * expected_scale;

	EXPECT_NEAR(distance, expected_distance, 1e-5);
}

TEST(ObstacleMathTest, GetBinAtAngle)
{
	uint16_t start_bin = 0;
	float bin_width = 5.0f;

	// GIVEN: a start bin, bin width, and angle
	float angle = 0.0f;

	// WHEN: we calculate the bin index at the angle
	uint16_t bin_index = ObstacleMath::get_bin_at_angle(start_bin, bin_width, angle);

	// THEN: the bin index should be correct
	EXPECT_EQ(bin_index, 0);

	// GIVEN: a start bin, bin width, and angle
	angle = 90.0f;

	// WHEN: we calculate the bin index at the angle
	bin_index = ObstacleMath::get_bin_at_angle(start_bin, bin_width, angle);

	// THEN: the bin index should be correct
	EXPECT_EQ(bin_index, 18);

	// GIVEN: a start bin, bin width, and angle
	angle = -90.0f;

	// WHEN: we calculate the bin index at the angle
	bin_index = ObstacleMath::get_bin_at_angle(start_bin, bin_width, angle);

	// THEN: the bin index should be correct
	EXPECT_EQ(bin_index, 54);

	// GIVEN: a start bin, bin width, and angle
	angle = 450.0f;

	// WHEN: we calculate the bin index at the angle
	bin_index = ObstacleMath::get_bin_at_angle(start_bin, bin_width, angle);

	// THEN: the bin index should be correct
	EXPECT_EQ(bin_index, 18);
}


TEST(ObstacleMathTest, OffsetBinIndex)
{
	// GIVEN: a bin index, bin width, and angle offset
	uint16_t bin = 0;
	float bin_width = 5.0f;
	float angle_offset = -120.0f;

	// WHEN: we offset the bin index
	uint16_t new_bin_index = ObstacleMath::get_offset_bin_index(bin, bin_width, angle_offset);

	// THEN: the new bin index should be correct
	EXPECT_EQ(new_bin_index, 24);

	// GIVEN: a bin index, bin width, and angle offset
	bin = 24;
	bin_width = 5.0f;
	angle_offset = 120.0f;

	// WHEN: we offset the bin index
	new_bin_index = ObstacleMath::get_offset_bin_index(bin, bin_width, angle_offset);

	// THEN: the new bin index should be correct
	EXPECT_EQ(new_bin_index, 0);
}
