#pragma once

#include <iostream>

using namespace std;

namespace RSCalibration
{
	const static double MARKER_SIDE = 0.015;
	const static int TIMES = 6;
	const static int CAMERAS = 4;
	const static int MARKERS = 11;
	const static int BASE_MARKER_ID = 0;

	const static string SERIAL_NUMBERS[4] = { "821312061029", "816612062327", "821212062536", "821212061326" };
	const static int MARKER_IDS[11] = { 0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 23 };
}