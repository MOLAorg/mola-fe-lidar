/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-mola-fe-lidar-3d-align.cpp
 * @brief  test for 3D lidar scan alignment
 * @author Jose Luis Blanco Claraco
 * @date   Jan 24, 2019
 */

#include <mola-fe-lidar-3d/LidarOdometry3D.h>
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
static TCLAP::CmdLine cmd("test-mola-fe-lidar-3d-align");

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
    using namespace std::string_literals;

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
    mola::LidarOdometry3D module;

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

    mola::LidarOdometry3D::lidar_scan_t pcs1;
    {
        mrpt::system::CTimeLoggerEntry tle(timlog, "filterPointCloud");
        pcs1 = module.filterPointCloud(*pc1);
    }

    mola::LidarOdometry3D::lidar_scan_t pcs2;
    {
        mrpt::system::CTimeLoggerEntry tle(timlog, "filterPointCloud");
        pcs2 = module.filterPointCloud(*pc2);
    }

    // Send to ICP all layers except "raw":
    mola::LidarOdometry3D::ICP_Input icp_in;
    icp_in.to_pc   = mola::LidarOdometry3D::lidar_scan_t::Create(pcs2);
    icp_in.from_pc = mola::LidarOdometry3D::lidar_scan_t::Create(pcs1);

    // Add raw now, afterwards:
    pcs2.pc.point_layers["raw"] = pc2;
    pcs1.pc.point_layers["raw"] = pc1;

    // Select ICP configuration parameter set:
    switch (arg_icp_params_set.getValue())
    {
        case 0:
            icp_in.icp_params = module.params_.icp_params_with_vel;
            icp_in.align_kind = mola::LidarOdometry3D::AlignKind::LidarOdometry;
            break;
        case 1:
            icp_in.icp_params = module.params_.icp_params_without_vel;
            icp_in.align_kind = mola::LidarOdometry3D::AlignKind::NearbyAlign;
            break;
        case 2:
            icp_in.icp_params = module.params_.icp_params_loopclosure;
            icp_in.align_kind = mola::LidarOdometry3D::AlignKind::LoopClosure;
            break;
        default:
            throw std::invalid_argument("icp-params-set: invalid value.");
    }

    mola::LidarOdometry3D::ICP_Output icp_out;
    module.setVerbosityLevel(mrpt::system::LVL_DEBUG);
    {
        mrpt::system::CTimeLoggerEntry tle(timlog, "run_one_icp");
        module.run_one_icp(icp_in, icp_out);
    }

    // Display "layers":
    std::map<std::string, mrpt::gui::CDisplayWindow3D::Ptr> wins;
    int                                                     x = 5, y = 5;
    for (const auto& layer : pcs1.pc.point_layers)
    {
        const auto name = layer.first;

        auto& win = wins[name] = mrpt::gui::CDisplayWindow3D::Create(name);

        mrpt::opengl::COpenGLScene::Ptr scene;

        {
            mrpt::gui::CDisplayWindow3DLocker lck(*win, scene);

            scene->clear();
            // XYZ corner at origin (map 1)
            scene->insert(
                mrpt::opengl::stock_objects::CornerXYZSimple(1.0f, 6.0f));
            // XYZ corner at aligned pose (map 2)
            {
                auto gl_corner2 =
                    mrpt::opengl::stock_objects::CornerXYZSimple(1.0f, 3.0f);
                gl_corner2->setPose(icp_out.found_pose_to_wrt_from.mean);
                scene->insert(gl_corner2);
            }

            auto gl_pc1 = mrpt::opengl::CSetOfObjects::Create();
            layer.second->renderOptions.color = mrpt::img::TColorf(1, 0, 0);
            layer.second->getAs3DObject(gl_pc1);
            scene->insert(gl_pc1);

            auto gl_pc2 = mrpt::opengl::CSetOfObjects::Create();
            pcs2.pc.point_layers.at(layer.first)->renderOptions.color =
                mrpt::img::TColorf(0, 0, 1);
            pcs2.pc.point_layers.at(layer.first)->getAs3DObject(gl_pc2);
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

    // Display "planes":
    {
        const auto name = "planes"s;

        auto& win = wins[name] = mrpt::gui::CDisplayWindow3D::Create(name);
        mrpt::opengl::COpenGLScene::Ptr   scene;
        mrpt::gui::CDisplayWindow3DLocker lck(*win, scene);

        scene->clear();
        scene->insert(mrpt::opengl::stock_objects::CornerXYZSimple(1.0f, 4.0f));

        // Overlay the raw points:
#if 0
        if (scan.pc.point_layers["raw"])
        {
            auto gl_pc = mrpt::opengl::CSetOfObjects::Create();
            scan.pc.point_layers["raw"]->getAs3DObject(gl_pc);
            scene->insert(gl_pc);
        }
#endif

        {
            p2p2::PointsPlanesICP::render_params_t pl1_render;
            pl1_render.plane_color = mrpt::img::TColor(0xff, 0x00, 0x00);
            pl1_render.plane_half_width =
                module.params_.voxel_filter_resolution * 0.5f;
            pl1_render.plane_grid_spacing = pl1_render.plane_half_width * 0.45f;

            auto gl_planes1 = mrpt::opengl::CSetOfObjects::Create();
            pcs1.pc.planesAsRenderizable(*gl_planes1, pl1_render);
            scene->insert(gl_planes1);
        }

        {
            p2p2::PointsPlanesICP::render_params_t pl2_render;
            pl2_render.plane_color = mrpt::img::TColor(0x00, 0x00, 0xff);
            pl2_render.plane_half_width =
                module.params_.voxel_filter_resolution * 0.5f;
            pl2_render.plane_grid_spacing = pl2_render.plane_half_width * 0.45f;

            auto gl_planes2 = mrpt::opengl::CSetOfObjects::Create();
            pcs2.pc.planesAsRenderizable(*gl_planes2, pl2_render);
            gl_planes2->setPose(icp_out.found_pose_to_wrt_from.mean);
            scene->insert(gl_planes2);
        }

        auto msg = mrpt::format(
            "layer=`%s`  => %u elements.", name.c_str(),
            static_cast<unsigned int>(pcs1.pc.planes.size()));
        win->addTextMessage(
            5, 5, msg, mrpt::img::TColorf(1, 1, 1), "sans", 10.0);

        win->setPos(x, y);
        y += 350;

        win->repaint();
    }

    std::cout << "Align results:\n"
                 " - found_pose_to_wrt_from:"
              << icp_out.found_pose_to_wrt_from.asString() << "\n"
              << " - goodness: " << icp_out.goodness << "\n";

    // Evaluate with the raw point clouds:
    {
        // Matching params for point-to-point:
        mrpt::maps::TMatchingParams mp;
        mp.maxDistForCorrespondence        = 0.20f;
        mp.maxAngularDistForCorrespondence = 0;
        mp.onlyKeepTheClosest              = true;
        mp.decimation_other_map_points     = 1;
        mp.offset_other_map_points         = 0;

        mrpt::tfest::TMatchingPairList    mpl;
        mrpt::maps::TMatchingExtraResults mres;

        pc1->determineMatching3D(
            pc2.get(), icp_out.found_pose_to_wrt_from.mean, mpl, mp, mres);

        std::cout << "Validation match against raw point cloud:\n Match ratio: "
                  << mres.correspondencesRatio * 100 << "%\n"
                  << " RMSE: " << std::sqrt(mres.sumSqrDist / mpl.size())
                  << "\n"
                  << " Number of matched points: " << mpl.size() << "\n";
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
