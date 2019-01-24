/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-mola-fe-lidar-icp-segment-scan.cpp
 * @brief  test for 3D lidar scan segmentation
 * @author Jose Luis Blanco Claraco
 * @date   Jan 24, 2019
 */

#include <mola-fe-lidar-icp/LidarICP.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/otherlibs/tclap/CmdLine.h>

// Declare supported cli switches ===========
static TCLAP::CmdLine cmd("test-mola-fe-lidar-icp-segment-scan");

static TCLAP::ValueArg<std::string> arg_lidar_kitti_file(
    "", "input-kitti-file", "Load 3D scan from a Kitti lidar (.bin) file", true,
    "", "./00000.bin", cmd);

int main(int argc, char** argv)
{
    try
    {
        // Parse arguments:
        if (!cmd.parse(argc, argv)) return 1;  // should exit.

        return 0;
    }
    catch (std::exception& e)
    {
        std::cerr << "Exit due to exception:\n"
                  << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
