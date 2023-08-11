#ifndef TESSERACT_RASTER_DEMO_PLANNING_SERVER_H
#define TESSERACT_RASTER_DEMO_PLANNING_SERVER_H

#include <tesseract_common/manipulator_info.h>
#include <tesseract_environment/environment.h>

namespace planning_server
{

bool run(tesseract_environment::Environment::Ptr env, tesseract_common::ManipulatorInfo manipulator, bool debug);

}

#endif
