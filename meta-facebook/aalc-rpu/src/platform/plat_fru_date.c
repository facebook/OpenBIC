/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "libutil.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_fru_date);

#define MINUTES_IN_HOUR 60
#define HOURS_IN_DAY 24
#define DAYS_IN_YEAR 365
#define LEAP_YEAR_CYCLE 4
#define MINUTES_FROM_1970_1996 (820454400 / 60) // Convert seconds to minutes

// Determine if a year is a leap year
int is_leap_year(int year)
{
	return (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
}

// Get the number of days in a specific month of a given year
int days_in_month(int year, int month)
{
	const int days[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
	if (month == 2 && is_leap_year(year)) {
		return 29;
	}
	return days[month - 1];
}

// Convert a string to its ASCII hexadecimal representation and store it in the result array
void store_ascii_to_result(char *str, uint8_t *result, int *index)
{
	for (int i = 0; str[i] != '\0'; i++) {
		result[(*index)++] =
			(uint8_t)str[i]; // Convert character to ASCII value and store it
	}
}

// Convert the year, month, day, hour, minute, etc., to strings and store them in the result array
uint8_t store_time_to_result(int year, int month, int day, int hour, int minute, int weekday,
			     uint8_t *result)
{
	char year_str[5], month_str[3], day_str[3], hour_str[3], minute_str[3], weekday_str[2];
	int index = 0;

	// Convert year to string (e.g., 2024 -> "2024")
	snprintf(year_str, 5, "%04d", year);
	// Convert month, day, hour, minute to two-digit strings
	snprintf(month_str, 3, "%02d", month);
	snprintf(day_str, 3, "%02d", day);
	snprintf(hour_str, 3, "%02d", hour);
	snprintf(minute_str, 3, "%02d", minute);
	// Convert weekday to a one-digit string
	snprintf(weekday_str, 2, "%d", weekday);

	// Convert these strings to ASCII and store them in the result array
	store_ascii_to_result(year_str, result, &index); // Store year
	store_ascii_to_result(month_str, result, &index); // Store month
	store_ascii_to_result(day_str, result, &index); // Store day
	store_ascii_to_result(hour_str, result, &index); // Store hour
	store_ascii_to_result(minute_str, result, &index); // Store minute
	store_ascii_to_result(weekday_str, result, &index); // Store weekday

	// If the result is less than 16 bytes, pad with 0x00
	while (index < 16) {
		result[index++] = 0x00;
	}

	return true;
}

// Calculate the date and time from the number of minutes since the base year and store it as ASCII hexadecimal values in the result array
uint8_t calculate_time_in_hex(long long total_minutes, uint8_t *result)
{
	int year = 1970, month, day, hour, minute, weekday;
	long long days_elapsed = total_minutes / (HOURS_IN_DAY * MINUTES_IN_HOUR);
	long long remaining_minutes = total_minutes % (HOURS_IN_DAY * MINUTES_IN_HOUR);

	// Calculate the year
	while (days_elapsed >= (is_leap_year(year) ? 366 : 365)) {
		days_elapsed -= is_leap_year(year) ? 366 : 365;
		year++;
	}

	// Calculate the month and day
	month = 1;
	while (days_elapsed >= days_in_month(year, month)) {
		days_elapsed -= days_in_month(year, month);
		month++;
	}

	day = days_elapsed + 1;
	hour = remaining_minutes / MINUTES_IN_HOUR;
	minute = remaining_minutes % MINUTES_IN_HOUR;

	// Calculate the weekday (January 1, 1970, was a Thursday, so add 4)
	weekday = (total_minutes / (HOURS_IN_DAY * MINUTES_IN_HOUR) + 4) % 7;

	// Store the time result as ASCII hexadecimal values in the result array
	if (!store_time_to_result(year, month, day, hour, minute, weekday, result)) {
		LOG_ERR("Failed to store time to Result\n");
		return false;
	}

	printf("store time to Result: ");
	for (int i = 0; i < 16; i++) {
		printf("%02X ", result[i]);
	}

	return true;
}

uint8_t get_fru_date(uint16_t *data, uint16_t *return_data)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	long long total_minutes;
	uint8_t result[16]; // Fixed size array of 16 bytes
	//data have 3 bytes, need to combine in one and LSB
	printf("data: %x-%x-%x\n", data[0], data[1], data[2]);
	total_minutes = (data[2] << 16) | (data[1] << 8) | data[0];
	printf("read minutes: %llx\n", total_minutes);

	total_minutes += MINUTES_FROM_1970_1996;
	printf("Total minutes: %lld\n", total_minutes);
	// Calculate the time and store it in the result array
	if (!calculate_time_in_hex(total_minutes, result)) {
		LOG_ERR("calculate time in hex failed\n");
		return false;
	}

	// Output the result array
	printf("Result: ");
	for (int i = 0; i < 16; i++) {
		printf("%02X ", result[i]);
	}
	printf("\n");

	memcpy(return_data, result, sizeof(result));

	return true;
}
