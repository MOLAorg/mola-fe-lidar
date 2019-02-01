/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-mola-fe-lidar-icp-align.cpp
 * @brief  test for 3D lidar scan alignment
 * @author Jose Luis Blanco Claraco
 * @date   Jan 24, 2019
 */

#include <mola-fe-lidar-icp/LidarICP.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/otherlibs/tclap/CmdLine.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/filesystem.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

// Declare supported cli switches ===========
static TCLAP::CmdLine cmd("test-mola-fe-lidar-icp-align");

static TCLAP::ValueArg<std::string> arg_kitti_file1(
    "", "k1", "Scan1: Load 3D scan from a Kitti lidar (.bin) file", true, "",
    "./00001.bin", cmd);

static TCLAP::ValueArg<std::string> arg_kitti_file2(
    "", "k2", "Scan2: Load 3D scan from a Kitti lidar (.bin) file", true, "",
    "./00002.bin", cmd);

static TCLAP::ValueArg<std::string> arg_params_file(
    "c", "config-file",
    "Load parameters from a YAML config file, containing a top-level YAML "
    "entry named `params` with all the parameters under it",
    true, "", "config.yml", cmd);

static TCLAP::ValueArg<int> arg_icp_params_set(
    "", "params-set",
    "Set of ICP parameters to use: 0=with-vel-mode; 1=without-vel; "
    "2=loop-closure",
    true, 0, "0", cmd);

static mrpt::system::CTimeLogger timlog;

void do_scan_align_test()
{
    // Load input point cloud:
    auto       pc1  = mrpt::maps::CPointsMapXYZI::Create();
    const auto fil1 = arg_kitti_file1.getValue();
    std::cout << "Loading: " << fil1 << "\n";
    pc1->loadFromKittiVelodyneFile(fil1);
    std::cout << "Done. " << pc1->size() << " points.\n";

    auto       pc2  = mrpt::maps::CPointsMapXYZI::Create();
    const auto fil2 = arg_kitti_file2.getValue();
    std::cout << "Loading: " << fil2 << "\n";
    pc2->loadFromKittiVelodyneFile(fil2);
    std::cout << "Done. " << pc2->size() << " points.\n";

    // Filter:
    mola::LidarICP module;

    // Not needed outside of a real SLAM system:
    // module.initialize_common();

    // Load params:
    const auto cfg_file = arg_params_file.getValue();
    ASSERT_FILE_EXISTS_(cfg_file);

    std::cout << "Loading param file: " << cfg_file << "\n";
    const auto cfg = YAML::LoadFile(cfg_file);
    std::cout << "Done.\n";
    std::string str_params;
    {
        std::stringstream ss;
        ss << cfg;
        str_params = ss.str();
    }
    std::cout << "Initializing with these params:\n" << str_params << "\n";

    module.initialize(str_params);

    mola::LidarICP::pointclouds_t pcs1;
    pcs1.layers["original"] = pc1;
    {
        mrpt::system::CTimeLoggerEntry tle(timlog, "filterPointCloud");
        module.filterPointCloud(pcs1);
    }

    mola::LidarICP::pointclouds_t pcs2;
    pcs2.layers["original"] = pc2;
    {
        mrpt::system::CTimeLoggerEntry tle(timlog, "filterPointCloud");
        module.filterPointCloud(pcs2);
    }

    // Send to ICP all layers except "original":
    mola::LidarICP::ICP_Input icp_in;

    switch (arg_icp_params_set.getValue())
    {
        case 0:
            icp_in.icp_params = module.params_.icp_params_with_vel;
            icp_in.align_kind = mola::LidarICP::AlignKind::LidarOdometry;
            break;
        case 1:
            icp_in.icp_params = module.params_.icp_params_without_vel;
            icp_in.align_kind = mola::LidarICP::AlignKind::NearbyAlign;
            break;
        case 2:
            icp_in.icp_params = module.params_.icp_params_loopclosure;
            icp_in.align_kind = mola::LidarICP::AlignKind::LoopClosure;
            break;
        default:
            throw std::invalid_argument("icp-params-set: invalid value.");
    }

    for (const auto& l : pcs1.layers)
        if (l.first.compare("original") != 0)
            icp_in.from_pc->layers[l.first] = l.second;
    for (const auto& l : pcs2.layers)
        if (l.first.compare("original") != 0)
            icp_in.to_pc->layers[l.first] = l.second;

    mola::LidarICP::ICP_Output icp_out;
    module.setVerbosityLevel(mrpt::system::LVL_DEBUG);
    {
        mrpt::system::CTimeLoggerEntry tle(timlog, "run_one_icp");
        module.run_one_icp(icp_in, icp_out);
    }

    // Display "layers":
    std::map<std::string, mrpt::gui::CDisplayWindow3D::Ptr> wins;
    int                                                     x = 5, y = 5;
    for (const auto& layer : pcs1.layers)
    {
        const auto name = layer.first;

        auto& win = wins[name] = mrpt::gui::CDisplayWindow3D::Create(name);

        mrpt::opengl::COpenGLScene::Ptr scene;

        {
            mrpt::gui::CDisplayWindow3DLocker lck(*win, scene);

            scene->clear();
            scene->insert(
                mrpt::opengl::stock_objects::CornerXYZSimple(1.0f, 4.0f));

            auto gl_pc1 = mrpt::opengl::CSetOfObjects::Create();
            layer.second->renderOptions.color = mrpt::img::TColorf(1, 0, 0);
            layer.second->getAs3DObject(gl_pc1);
            scene->insert(gl_pc1);

            auto gl_pc2 = mrpt::opengl::CSetOfObjects::Create();
            pcs2.layers.at(layer.first)->renderOptions.color =
                mrpt::img::TColorf(0, 0, 1);
            pcs2.layers.at(layer.first)->getAs3DObject(gl_pc2);
            gl_pc2->setPose(icp_out.found_pose_to_wrt_from.mean);
            scene->insert(gl_pc2);

            auto msg = mrpt::format(
                "layer=`%s`  => %u points.", name.c_str(),
                static_cast<unsigned int>(layer.second->size()));
            win->addTextMessage(
                5, 5, msg, mrpt::img::TColorf(1, 1, 1), "sans", 10.0);

            win->setPos(x, y);
            y += 350;
        }
        win->repaint();
    }

    std::cout << "Close windows or hit a key on first window to quit.\n";
    if (!wins.empty()) wins.begin()->second->waitForKey();
}

int main(int argc, char** argv)
{
    try
    {
        // Parse arguments:
        if (!cmd.parse(argc, argv)) return 1;  // should exit.

        do_scan_align_test();
        return 0;
    }
    catch (std::exception& e)
    {
        std::cerr << "Exit due to exception:\n"
                  << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
