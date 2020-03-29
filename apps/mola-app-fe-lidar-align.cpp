/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-mola-fe-lidar-align.cpp
 * @brief  test for 3D lidar scan alignment
 * @author Jose Luis Blanco Claraco
 * @date   Jan 24, 2019
 */

#include <mola-fe-lidar/LidarOdometry.h>
#include <mola-kernel/yaml_helpers.h>
#include <mola-lidar-segmentation/FilterEdgesPlanes.h>
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/filesystem.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

// Declare supported cli switches ===========
static TCLAP::CmdLine cmd("test-mola-fe-lidar-align");

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

static TCLAP::ValueArg<std::string> arg_init_pose(
    "p", "pose-guess", "Initial guess for the relative pose", false,
    "[0 0 0 0 0 0]", "(x,y,z [m] yaw pitch roll [deg])", cmd);

static TCLAP::SwitchArg arg_no_gui(
    "", "no-gui", "Disables the gui (Default: NO)", cmd);

// clang-format off
static TCLAP::ValueArg<std::string> arg_lidar_pose(
    "", "lidar-pose",
    "Defines the 4x4 homogeneous matrix of the LiDAR sensor in the vehicle "
    "frame.\n"
    "Use Matlab format, quoted as a string. For example:\n"
    "For KAIST left_VLP :\n"
    " --lidar-pose \"[-5.1406e-01 -7.0220e-01 -4.9259e-01 -4.4069e-01;"
    "           4.8648e-01 -7.1167e-01 5.0680e-01 3.9705e-01 ;"
    "          -7.0644e-01  2.0893e-02 7.0745e-01 1.9095e+00 ;"
    "           0 0 0 1 ]\"\n"
    "For KAIST right_VLP:\n"
    " --lidar-pose \"[-5.1215e-01 6.9924e-01 -4.9876e-01 -4.4988e-01;"
    "      -4.9481e-01 -7.1485e-01 -4.9412e-01 -4.1671e-01;"
    "      -7.0204e-01 -6.2641e-03 7.1210e-01 1.9129e+00;"
    "      0 0 0 1 ]\"\n",
    false, "",
    "[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]", cmd);
// clang-format on

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

    // Set in the coordinate frame of the vehicle:
    if (arg_lidar_pose.isSet())
    {
        mrpt::math::CMatrixDouble44 HM;
        const auto                  sMat = arg_lidar_pose.getValue();
        if (!HM.fromMatlabStringFormat(sMat))
        {
            THROW_EXCEPTION_FMT(
                "Malformed matlab-like 4x4 homogeneous matrix: `%s`",
                sMat.c_str());
        }
        auto p = mrpt::poses::CPose3D(HM);
        std::cout << "Using sensor pose: " << p.asString() << "\n";
        pc1->changeCoordinatesReference(p);
        pc2->changeCoordinatesReference(p);
    }

    // Load params:
    const auto cfg_file = arg_params_file.getValue();
    ASSERT_FILE_EXISTS_(cfg_file);

    std::cout << "Loading param file: " << cfg_file << "\n";
    const auto cfg = YAML::LoadFile(cfg_file);
    std::cout << "Done.\n";
    std::string str_params = mola::yaml2string(cfg);
    std::cout << "Initializing with these params:\n" << str_params << "\n";

    mola::LidarOdometry module;
    module.initialize(str_params);

    MRPT_TODO(
        "Convert to class factory instead of reusing mola::LidarOdometry?");
    // then, do: filter->initialize(xxx);

    auto filter = module.stateCopy().pc_filter;
    filter->setMinLoggingLevel(mrpt::system::LVL_DEBUG);

    // Filter pc #1:
    auto raw_input1        = mrpt::obs::CObservationPointCloud::Create();
    raw_input1->pointcloud = pc1;
    auto pcs1              = mp2p_icp::pointcloud_t::Create();

    mrpt::system::CTimeLoggerEntry tle1(timlog, "filterPointCloud");
    filter->filter(raw_input1, *pcs1);
    tle1.stop();

    // Filter pc #2:
    auto raw_input2        = mrpt::obs::CObservationPointCloud::Create();
    raw_input2->pointcloud = pc2;
    auto pcs2              = mp2p_icp::pointcloud_t::Create();

    mrpt::system::CTimeLoggerEntry tle2(timlog, "filterPointCloud");
    filter->filter(raw_input2, *pcs2);
    tle2.stop();

    // Send to ICP all layers except "original":
    mola::LidarOdometry::ICP_Input icp_in;

    icp_in.from_pc = pcs1;
    icp_in.to_pc   = pcs2;

    // Select ICP configuration parameter set:
    switch (arg_icp_params_set.getValue())
    {
        case 0:
            icp_in.icp_params = module.params_.icp_params_with_vel;
            icp_in.align_kind = mola::LidarOdometry::AlignKind::LidarOdometry;
            break;
        case 1:
            icp_in.icp_params = module.params_.icp_params_without_vel;
            icp_in.align_kind = mola::LidarOdometry::AlignKind::NearbyAlign;
            break;
        case 2:
            icp_in.icp_params = module.params_.icp_params_loopclosure;
            icp_in.align_kind = mola::LidarOdometry::AlignKind::LoopClosure;
            break;
        default:
            throw std::invalid_argument("icp-params-set: invalid value.");
    }

    // Set initial guess:
    icp_in.init_guess_to_wrt_from.fromString(arg_init_pose.getValue());

    mola::LidarOdometry::ICP_Output icp_out;
    module.setVerbosityLevel(mrpt::system::LVL_DEBUG);
    {
        mrpt::system::CTimeLoggerEntry tle(timlog, "run_one_icp");
        module.run_one_icp(icp_in, icp_out);
    }

    std::cout << "Align results:\n"
                 " - found_pose_to_wrt_from:"
              << icp_out.found_pose_to_wrt_from.asString() << "\n"
              << " - goodness: " << icp_out.goodness << "\n";

    if (arg_no_gui.isSet()) return;

    // Display "layers":
    std::map<std::string, mrpt::gui::CDisplayWindow3D::Ptr> wins;
    int                                                     x = 5, y = 5;
    for (const auto& layer : pcs1->point_layers)
    {
        const auto name = layer.first;

        auto& win = wins[name] = mrpt::gui::CDisplayWindow3D::Create(name);

        mrpt::opengl::COpenGLScene::Ptr scene;

        {
            mrpt::gui::CDisplayWindow3DLocker lck(*win, scene);

            scene->clear();
            scene->insert(
                mrpt::opengl::stock_objects::CornerXYZSimple(1.0f, 4.0f));

            {
                auto corner2 =
                    mrpt::opengl::stock_objects::CornerXYZSimple(1.0f, 3.0f);
                corner2->setPose(icp_out.found_pose_to_wrt_from.mean);
                scene->insert(corner2);
            }

            auto gl_pc1 = mrpt::opengl::CSetOfObjects::Create();
            layer.second->renderOptions.color = mrpt::img::TColorf(1, 0, 0);
            layer.second->getAs3DObject(gl_pc1);
            scene->insert(gl_pc1);

            auto gl_pc2 = mrpt::opengl::CSetOfObjects::Create();
            pcs2->point_layers.at(layer.first)->renderOptions.color =
                mrpt::img::TColorf(0, 0, 1);
            pcs2->point_layers.at(layer.first)->getAs3DObject(gl_pc2);
            gl_pc2->setPose(icp_out.found_pose_to_wrt_from.mean);
            scene->insert(gl_pc2);

            auto msg = mrpt::format(
                "layer=`%s`  => %u points.", name.c_str(),
                static_cast<unsigned int>(layer.second->size()));
            win->addTextMessage(5, 5, msg);

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
        {
            auto corner2 =
                mrpt::opengl::stock_objects::CornerXYZSimple(1.0f, 3.0f);
            corner2->setPose(icp_out.found_pose_to_wrt_from.mean);
            scene->insert(corner2);
        }

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
            mp2p_icp::render_params_t pl1_render;
            pl1_render.plane_color = mrpt::img::TColor(0xff, 0x00, 0x00);
            pl1_render.plane_half_width =
                module.params_.voxel_filter_resolution * 0.5f;
            pl1_render.plane_grid_spacing = pl1_render.plane_half_width * 0.45f;

            auto gl_planes1 = mrpt::opengl::CSetOfObjects::Create();
            pcs1->planesAsRenderizable(*gl_planes1, pl1_render);
            scene->insert(gl_planes1);
        }

        {
            mp2p_icp::render_params_t pl2_render;
            pl2_render.plane_color = mrpt::img::TColor(0x00, 0x00, 0xff);
            pl2_render.plane_half_width =
                module.params_.voxel_filter_resolution * 0.5f;
            pl2_render.plane_grid_spacing = pl2_render.plane_half_width * 0.45f;

            auto gl_planes2 = mrpt::opengl::CSetOfObjects::Create();
            pcs2->planesAsRenderizable(*gl_planes2, pl2_render);
            gl_planes2->setPose(icp_out.found_pose_to_wrt_from.mean);
            scene->insert(gl_planes2);
        }

        auto msg = mrpt::format(
            "layer=`%s`  => %u elements.", name.c_str(),
            static_cast<unsigned int>(pcs1->planes.size()));
        win->addTextMessage(5, 5, msg);

        win->setPos(x, y);
        y += 350;

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
