#pragma once

#include <iostream>

#include <ceres/ceres.h>
#include <opencv2/opencv.hpp>

#include "my_const.h"
#include "correspondencer.h"
#include "my_struct.h"
#include "my_io.h"
#include "bundle_adjustment_manager.h"
#include "reprojection_check.h"

using namespace std;
using namespace cv;
using namespace RSCalibration;