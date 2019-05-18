/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   LidarOdometry3D.cpp
 * @brief  Simple SLAM FrontEnd for point-cloud sensors via ICP registration
 * @author Jose Luis Blanco Claraco
 * @date   Dec 17, 2018
 */

/** \defgroup mola_fe_lidar_icp_grp mola-fe-lidar-3d.
 * Simple SLAM FrontEnd for point-cloud sensors via ICP registration.
 *
 *
 */

#include <mola-fe-lidar-3d/LidarOdometry3D.h>
#include <mola-kernel/yaml_helpers.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/core/initializer.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <yaml-cpp/yaml.h>

using namespace mola;

static const std::string ANNOTATION_NAME_PC_LAYERS = "lidar-pointcloud-layers";

MRPT_INITIALIZER(do_register)
{
    // Register MOLA modules:
    MOLA_REGISTER_MODULE(LidarOdometry3D)

    // Register serializable classes:
    mrpt::rtti::registerClass(CLASS_ID(mola::LidarOdometry3D::lidar_scan_t));
}

IMPLEMENTS_SERIALIZABLE(lidar_scan_t, CSerializable, mola::LidarOdometry3D)

//
uint8_t LidarOdometry3D::lidar_scan_t::serializeGetVersion() const { return 0; }
void    LidarOdometry3D::lidar_scan_t::serializeTo(
    mrpt::serialization::CArchive& out) const
{
    // out.WriteAs<uint32_t>(layers.size());
    // for (const auto& l : layers) out << l.first << l.second;
    MRPT_TODO("Implement");
    // out << pc.point_layers << pc.lines << pc.planes;
}
void LidarOdometry3D::lidar_scan_t::serializeFrom(
    mrpt::serialization::CArchive& in, uint8_t version)
{
    switch (version)
    {
        case 0:
        {
            MRPT_TODO("Implement");
            // in >> pc.point_layers >> pc.lines >> pc.planes;
        }
        break;
        default:
            MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
    };
}

LidarOdometry3D::LidarOdometry3D() = default;

static void load_icp_set_of_params(
    std::vector<p2p2::Parameters>& out, YAML::Node& cfg,
    const std::string& prefix)
{
    MRPT_START

    using namespace std::string_literals;

    std::string sName = prefix + "maxIterations"s;
    ASSERTMSG_(cfg[sName], "Missing YAML required entry: `"s + sName + "`"s);
    std::string maxIterations = cfg[sName].as<std::string>("");

    sName = prefix + "thresholdDist"s;
    ASSERTMSG_(cfg[sName], "Missing YAML required entry: `"s + sName + "`"s);
    std::string thresholdDists = cfg[sName].as<std::string>("");

    sName = prefix + "thresholdAng"s;
    ASSERTMSG_(cfg[sName], "Missing YAML required entry: `"s + sName + "`"s);
    std::string thresholdAngs = cfg[sName].as<std::string>("");

    sName = prefix + "useRobustKernel"s;
    ASSERTMSG_(cfg[sName], "Missing YAML required entry: `"s + sName + "`"s);
    std::string useKernel = cfg[sName].as<std::string>("");

    // Vector -> values:
    std::vector<std::string> maxIterations_vals;
    mrpt::system::tokenize(maxIterations, " ,", maxIterations_vals);

    std::vector<std::string> thresholdDists_vals;
    mrpt::system::tokenize(thresholdDists, " ,", thresholdDists_vals);

    std::vector<std::string> thresholdAngs_vals;
    mrpt::system::tokenize(thresholdAngs, " ,", thresholdAngs_vals);

    std::vector<std::string> useKernel_vals;
    mrpt::system::tokenize(useKernel, " ,", useKernel_vals);

    ASSERT_(maxIterations_vals.size() >= 1);
    ASSERT_(maxIterations_vals.size() == thresholdDists_vals.size());
    ASSERT_(maxIterations_vals.size() == thresholdAngs_vals.size());
    ASSERT_(maxIterations_vals.size() == useKernel_vals.size());

    const size_t n = thresholdAngs_vals.size();
    out.resize(n);
    for (size_t i = 0; i < n; i++)
    {
        out[i].maxIterations = std::stoul(maxIterations_vals[i]);
        out[i].thresholdDist = std::stod(thresholdDists_vals[i]);
        out[i].thresholdAng  = mrpt::DEG2RAD(std::stod(thresholdAngs_vals[i]));
        out[i].use_kernel    = (std::stoul(useKernel_vals[i]) != 0);
    }

    MRPT_END
}

void LidarOdometry3D::initialize(const std::string& cfg_block)
{
    MRPT_TRY_START

    // Load:
    auto c   = YAML::Load(cfg_block);
    auto cfg = c["params"];
    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << cfg);

    YAML_LOAD_REQ(params_, min_dist_xyz_between_keyframes, double);
    YAML_LOAD_OPT_DEG(params_, min_rotation_between_keyframes, double);

    YAML_LOAD_OPT(params_, min_time_between_scans, double);
    YAML_LOAD_OPT(params_, min_icp_goodness, double);
    YAML_LOAD_OPT(params_, min_icp_goodness_lc, double);
    YAML_LOAD_OPT(params_, max_correspondences_per_layer, unsigned int);

    YAML_LOAD_OPT(params_, voxel_filter4planes_resolution, double);
    YAML_LOAD_OPT(params_, voxel_filter4planes_min_point_count, unsigned int);
    YAML_LOAD_OPT(params_, voxel_filter4planes_min_e1_e0, float);
    YAML_LOAD_OPT(params_, voxel_filter4planes_min_e2_e0, float);

    YAML_LOAD_OPT(params_, voxel_filter4edges_resolution, double);
    YAML_LOAD_OPT(params_, voxel_filter4edges_min_point_count, unsigned int);
    YAML_LOAD_OPT(params_, voxel_filter4edges_max_e1_e0, float);
    YAML_LOAD_OPT(params_, voxel_filter4edges_min_e2_e1, float);
    YAML_LOAD_OPT(params_, voxel_filter4edges_decimation, unsigned int);

    YAML_LOAD_OPT(params_, min_dist_to_matching, double);
    YAML_LOAD_OPT(params_, max_dist_to_matching, double);
    YAML_LOAD_OPT(params_, max_dist_to_loop_closure, double);
    YAML_LOAD_OPT(params_, max_nearby_align_checks, unsigned int);
    YAML_LOAD_OPT(params_, min_topo_dist_to_consider_loopclosure, unsigned int);
    YAML_LOAD_OPT(params_, loop_closure_montecarlo_samples, unsigned int);

    YAML_LOAD_OPT(params_, viz_decor_decimation, int);
    YAML_LOAD_OPT(params_, viz_decor_pointsize, float);

    load_icp_set_of_params(
        params_.icp_params_with_vel, cfg, "icp_params_with_vel.");
    load_icp_set_of_params(
        params_.icp_params_without_vel, cfg, "icp_params_without_vel.");
    load_icp_set_of_params(
        params_.icp_params_loopclosure, cfg, "icp_params_loopclosure.");

    YAML_LOAD_OPT(params_, debug_save_lidar_odometry, bool);
    YAML_LOAD_OPT(params_, debug_save_extra_edges, bool);
    YAML_LOAD_OPT(params_, debug_save_loop_closures, bool);

    {
        ProfilerEntry tle(profiler_, "filterPointCloud_initialize");
        state_.filter_grid4planes.resize(
            {-90.0, -90.0, -10.0}, {90.0, 90.0, 10.0},
            params_.voxel_filter4planes_resolution);

        state_.filter_grid4edges.resize(
            {-90.0, -90.0, -10.0}, {90.0, 90.0, 10.0},
            params_.voxel_filter4edges_resolution);

        MRPT_TODO("make these a parameter");
        state_.filter_grid4planes.params_.min_consecutive_distance = 0.05f;
        state_.filter_grid4edges.params_.min_consecutive_distance  = 0.20f;
    }

    // attach to world model, if present:
    auto wms = findService<WorldModel>();
    if (wms.size() == 1)
        worldmodel_ = std::dynamic_pointer_cast<WorldModel>(wms[0]);

    MRPT_TRY_END
}
void LidarOdometry3D::spinOnce()
{
    MRPT_TRY_START

    ProfilerEntry tleg(profiler_, "spinOnce");

    //
    MRPT_TRY_END
}

void LidarOdometry3D::reset() { state_ = MethodState(); }

void LidarOdometry3D::onNewObservation(CObservation::Ptr& o)
{
    MRPT_TRY_START
    ProfilerEntry tleg(profiler_, "onNewObservation");

    // Only process "my" sensor source:
    ASSERT_(o);
    if (o->sensorLabel != raw_sensor_label_) return;

    const auto queued = worker_pool_.pendingTasks();
    profiler_.registerUserMeasure("onNewObservation.queue_length", queued);
    if (queued > 10)
    {
        MRPT_LOG_THROTTLE_ERROR(
            1.0, "Dropping observation due to worker threads too busy.");
        profiler_.registerUserMeasure("onNewObservation.drop_observation", 1);
        return;
    }
    profiler_.enter("delay_onNewObs_to_process");

    // Enqueue task:
    worker_pool_.enqueue(&LidarOdometry3D::doProcessNewObservation, this, o);

    MRPT_TRY_END
}

// here happens the main stuff:
void LidarOdometry3D::doProcessNewObservation(CObservation::Ptr& o)
{
    // All methods that are enqueued into a thread pool should have its own
    // top-level try-catch:
    try
    {
        ASSERT_(o);

        ProfilerEntry tleg(profiler_, "doProcessNewObservation");
        profiler_.leave("delay_onNewObs_to_process");

        // Only process pointclouds that are sufficiently apart in time:
        const auto this_obs_tim = o->timestamp;
        if (state_.last_obs_tim != mrpt::Clock::time_point() &&
            mrpt::system::timeDifference(state_.last_obs_tim, this_obs_tim) <
                params_.min_time_between_scans)
        {
            // Drop observation.
            MRPT_LOG_DEBUG(
                "doProcessNewObservation: dropping observation, for "
                "`min_time_between_scans`.");
            return;
        }

        // Extract points (and clasify them, extract features,...) from
        // observation:
        auto this_obs_points = lidar_scan_t::Create();
        bool have_points;
        {
            auto raw_pc = mrpt::maps::CSimplePointsMap::Create();

            {
                ProfilerEntry tle(
                    profiler_, "doProcessNewObservation.1.obs2pc");
                have_points = raw_pc->insertObservationPtr(o);
            }

            {
                ProfilerEntry tle(
                    profiler_, "doProcessNewObservation.2.filter_pointclouds");
                *this_obs_points = filterPointCloud(*raw_pc);
            }

            this_obs_points->pc.point_layers["raw"] = raw_pc;
        }

        profiler_.enter("doProcessNewObservation.2b.copy_vars");

        // Store for next step:
        auto last_obs_tim   = state_.last_obs_tim;
        auto last_points    = state_.last_points;
        state_.last_obs_tim = this_obs_tim;
        state_.last_points  = this_obs_points;

        profiler_.leave("doProcessNewObservation.2b.copy_vars");

        if (!have_points)
        {
            MRPT_LOG_WARN_STREAM(
                "Observation of type `" << o->GetRuntimeClass()->className
                                        << "` could not be converted into a "
                                           "pointcloud. Doing nothing.");
            return;
        }

        bool create_keyframe = false;

        // First time we cannot do ICP since we need at least two pointclouds:
        if (!last_points || last_points->pc.point_layers.empty())
        {
            // Skip ICP.
            MRPT_LOG_DEBUG("First pointcloud: skipping ICP.");

            // Still, create a first KF (at origin)
            create_keyframe = true;
        }
        else
        {
            // Register point clouds using ICP:
            // ------------------------------------
            profiler_.enter("doProcessNewObservation.2c.prepare_icp_in");

            mrpt::poses::CPose3DPDFGaussian initial_guess;
            // Use velocity model for the initial guess:
            double dt = .0;
            if (last_obs_tim != mrpt::Clock::time_point())
                dt = mrpt::system::timeDifference(last_obs_tim, this_obs_tim);

            ICP_Output icp_out;
            ICP_Input  icp_in;
            icp_in.init_guess_to_wrt_from = mrpt::math::TPose3D(
                state_.last_iter_twist.vx * dt, state_.last_iter_twist.vy * dt,
                state_.last_iter_twist.vz * dt, state_.last_iter_twist.wz * dt,
                0, 0);
            MRPT_TODO("do omega_xyz part!");

            icp_in.to_pc   = this_obs_points;
            icp_in.from_pc = last_points;
            icp_in.from_id = state_.last_kf;
            icp_in.to_id =
                mola::INVALID_ID;  // current data, not a new KF (yet)
            icp_in.debug_str = "lidar_odom";

            // If we don't have a valid twist estimation, use a larger ICP
            // correspondence threshold:
            icp_in.icp_params = state_.last_iter_twist_is_good
                                    ? params_.icp_params_with_vel
                                    : params_.icp_params_without_vel;

            profiler_.leave("doProcessNewObservation.2c.prepare_icp_in");

            // Run ICP:
            {
                ProfilerEntry tle(
                    profiler_, "doProcessNewObservation.3.icp_latest");

                run_one_icp(icp_in, icp_out);
            }
            const mrpt::poses::CPose3D rel_pose =
                icp_out.found_pose_to_wrt_from.getMeanVal();

            // Update velocity model:
            state_.last_iter_twist.vx = rel_pose.x() / dt;
            state_.last_iter_twist.vy = rel_pose.y() / dt;
            state_.last_iter_twist.vz = rel_pose.z() / dt;
            state_.last_iter_twist.wz = rel_pose.yaw() / dt;
            MRPT_TODO("do omega_xyz part!");

            state_.last_iter_twist_is_good = true;

            MRPT_LOG_DEBUG_STREAM(
                "Est.twist=" << state_.last_iter_twist.asString());
            MRPT_LOG_DEBUG_STREAM(
                "Time since last scan="
                << mrpt::system::formatTimeInterval(dt));

            // Create a new KF if the distance since the last one is large
            // enough:
            state_.accum_since_last_kf = state_.accum_since_last_kf + rel_pose;
            const double dist_eucl_since_last =
                state_.accum_since_last_kf.norm();
            const double rot_since_last = 0;
            MRPT_TODO("Add rotation threshold");

            MRPT_LOG_DEBUG_FMT(
                "Since last KF: dist=%5.03f m rotation=%.01f deg",
                dist_eucl_since_last, mrpt::RAD2DEG(rot_since_last));

            const bool far_enough =
                (dist_eucl_since_last >
                     params_.min_dist_xyz_between_keyframes ||
                 rot_since_last > params_.min_rotation_between_keyframes);

            if (far_enough)
            {
                if (icp_out.goodness > params_.min_icp_goodness)
                    create_keyframe = true;
                else
                {
                    MRPT_LOG_WARN_FMT(
                        "Cannot create KF due to bad quality ICP: "
                        "goodness=%.02f%%",
                        100.0 * icp_out.goodness);
                }
            }

        }  // end: yes, we can do ICP

        // Should we create a new KF?
        if (create_keyframe)
        {
            // Yes: create new KF
            // 1) New KeyFrame
            BackEndBase::ProposeKF_Input kf;

            kf.timestamp = this_obs_tim;
            MRPT_TODO("re-enable this");
            if (0)
            {
                mrpt::obs::CSensoryFrame& sf = kf.observations.emplace();
                sf.push_back(o);
            }

            profiler_.enter("doProcessNewObservation.3a.addKeyFrame");

            std::future<BackEndBase::ProposeKF_Output> kf_out_fut;
            kf_out_fut = slam_backend_->addKeyFrame(kf);

            // Wait until it's executed:
            auto kf_out = kf_out_fut.get();

            ASSERT_(kf_out.success);
            ASSERT_(kf_out.new_kf_id);

            const mola::id_t new_kf_id = kf_out.new_kf_id.value();
            ASSERT_(new_kf_id != mola::INVALID_ID);

            profiler_.leave("doProcessNewObservation.3a.addKeyFrame");

            // Add point cloud to the KF annotations in the map:
            // Also, add Rendering decorations for the map visualizer:
            {
                profiler_.enter("doProcessNewObservation.wait.ent.writelock");
                worldmodel_->entities_lock_for_write();
                profiler_.leave("doProcessNewObservation.wait.ent.writelock");

                ProfilerEntry tle(
                    profiler_,
                    "doProcessNewObservation.4.writePCsToWorldModel");

                worldmodel_->entity_annotations_by_id(new_kf_id).emplace(
                    std::piecewise_construct,
                    std::forward_as_tuple(ANNOTATION_NAME_PC_LAYERS),
                    std::forward_as_tuple(
                        this_obs_points, ANNOTATION_NAME_PC_LAYERS));

                MRPT_TODO(
                    "move this render stuff to its a low-prio worker thread?");
                if (auto obs_pc =
                        dynamic_cast<const mrpt::obs::CObservationPointCloud*>(
                            o.get());
                    obs_pc != nullptr && obs_pc->pointcloud &&
                    (state_.kf_decor_decim_cnt < 0 ||
                     ++state_.kf_decor_decim_cnt >
                         params_.viz_decor_decimation))
                {
                    state_.kf_decor_decim_cnt = 0;

                    auto obs_render = mrpt::opengl::CSetOfObjects::Create();
                    obs_pc->pointcloud->renderOptions.point_size =
                        params_.viz_decor_pointsize;
                    obs_pc->pointcloud->getAs3DObject(obs_render);

                    worldmodel_->entity_annotations_by_id(new_kf_id).emplace(
                        std::piecewise_construct,
                        std::forward_as_tuple("render_decoration"),
                        std::forward_as_tuple(obs_render, "render_decoration"));
                }

                worldmodel_->entities_unlock_for_write();
            }
            MRPT_LOG_INFO_STREAM("New KF: ID=" << new_kf_id);

            // 2) New SE(3) constraint between consecutive Keyframes:
            if (state_.last_kf != mola::INVALID_ID)
            {
                std::future<BackEndBase::AddFactor_Output> factor_out_fut;
                // Important: The "constant velocity model" factor is
                // automatically added by the SLAM module (if applicable). Here,
                // all we need to tell it is the SE(3) constraint, and the
                // KeyFrame timestamp:
                mola::FactorRelativePose3 fPose3(
                    state_.last_kf, new_kf_id,
                    state_.accum_since_last_kf.asTPose());

                fPose3.noise_model_diag_xyz_ = 0.10;
                fPose3.noise_model_diag_rot_ = mrpt::DEG2RAD(1.0);

                mola::Factor f = std::move(fPose3);
                factor_out_fut = slam_backend_->addFactor(f);

                // Wait until it's executed:
                auto factor_out = factor_out_fut.get();
                ASSERT_(factor_out.success);
                ASSERT_(
                    factor_out.new_factor_id &&
                    factor_out.new_factor_id != mola::INVALID_FID);

                // Append to local graph as well:
                {
                    std::lock_guard<std::mutex> lck(local_pose_graph_mtx);

                    state_.local_pose_graph.graph.insertEdgeAtEnd(
                        state_.last_kf, *kf_out.new_kf_id,
                        state_.accum_since_last_kf);
                }

                MRPT_LOG_DEBUG_STREAM(
                    "New FactorRelativePose3ConstVel: #"
                    << state_.last_kf << " <=> #" << new_kf_id
                    << ". rel_pose=" << state_.accum_since_last_kf.asString());
            }

            // Reset accumulators:
            state_.accum_since_last_kf = mrpt::poses::CPose3D();
            state_.last_kf             = new_kf_id;
        }  // end done add a new KF

        // In any case, publish to the SLAM BackEnd what's our **current**
        // vehicle pose, no matter if it's a keyframe or not:
        {
            ProfilerEntry tle(
                profiler_,
                "doProcessNewObservation.5.advertiseUpdatedLocalization");

            BackEndBase::AdvertiseUpdatedLocalization_Input new_loc;
            new_loc.timestamp    = this_obs_tim;
            new_loc.reference_kf = state_.last_kf;
            new_loc.pose         = state_.accum_since_last_kf.asTPose();

            std::future<void> adv_pose_fut =
                slam_backend_->advertiseUpdatedLocalization(new_loc);
        }

        // Now, let's try to align this new KF against a few past KFs as well.
        // we'll do it in separate threads, with priorities so the latest KFs
        // are always attended first:
        bool can_check_for_other_matches = true;
        {
            std::lock_guard<std::mutex> lck(local_pose_graph_mtx);
            can_check_for_other_matches =
                !state_.local_pose_graph.graph.edges.empty();
        }

        if (can_check_for_other_matches)
        {
            ProfilerEntry tle(
                profiler_, "doProcessNewObservation.6.checkForNearbyKFs");
            checkForNearbyKFs();
        }
    }
    catch (const std::exception& e)
    {
        MRPT_LOG_ERROR_STREAM("Exception:\n" << mrpt::exception_to_str(e));
    }
}

void LidarOdometry3D::checkForNearbyKFs()
{
    using namespace std::string_literals;

    MRPT_START

    // Run Dijkstra wrt to the last KF:
    using euclidean_dist_t = double;
    std::map<
        euclidean_dist_t, std::pair<mrpt::graphs::TNodeID, topological_dist_t>>
               KF_distances;
    mola::id_t current_kf_id{mola::INVALID_ID};
    {
        std::lock_guard<std::mutex> lck(local_pose_graph_mtx);

        auto& lpg     = state_.local_pose_graph.graph;
        current_kf_id = state_.last_kf;

        // Call Dijkstra, starting from the current_kf_id as root, and build a
        // spanning-tree to estimate the relative poses, and topological
        // distances, to all other nodes:
        lpg.root = current_kf_id;
        lpg.nodes.clear();
        lpg.nodes[lpg.root] = mrpt::poses::CPose3D::Identity();
        std::map<mrpt::graphs::TNodeID, size_t> topolog_dists;

        lpg.dijkstra_nodes_estimate(std::ref(topolog_dists));

        // Sort KFs by distance
        for (const auto& kfs : lpg.nodes)
        {
            const auto it_dist = topolog_dists.find(kfs.first);
            ASSERT_(it_dist != topolog_dists.end());
            const topological_dist_t topo_d = it_dist->second;

            KF_distances[kfs.second.norm()] = std::make_pair(kfs.first, topo_d);
        }

        std::map<mrpt::graphs::TNodeID, std::set<mrpt::graphs::TNodeID>> adj;
        lpg.getAdjacencyMatrix(adj);

        // Remove too distant KFs:
        while (lpg.nodes.size() > params_.max_KFs_local_graph)
        {
            const auto id_to_remove = KF_distances.rbegin()->second.first;
            KF_distances.erase(std::prev(KF_distances.end()));

            lpg.nodes.erase(id_to_remove);
            for (const auto other_id : adj[id_to_remove])
            {
                lpg.edges.erase(std::make_pair(id_to_remove, other_id));
                lpg.edges.erase(std::make_pair(other_id, id_to_remove));
            }
        }
    }

    // Pick the node at an intermediary distance and try to align
    // against it:
    auto it1 = KF_distances.lower_bound(params_.min_dist_to_matching);
    auto it2 = KF_distances.upper_bound(std::max(
        params_.max_dist_to_loop_closure, params_.max_dist_to_matching));

    // Store here all the desired checks:
    std::vector<ICP_Input::Ptr>                nearby_checks;
    std::map<euclidean_dist_t, ICP_Input::Ptr> loop_closure_checks;

    for (auto it = it1; it != it2; ++it)
    {
        const double             kf_eucl_dist        = it->first;
        const auto               kf_id               = it->second.first;
        const topological_dist_t kf_topo_d           = it->second.second;
        bool                     edge_already_exists = false;
        const bool               is_potential_loop_closure =
            (kf_topo_d >= params_.min_topo_dist_to_consider_loopclosure);

        // Only explore KFs farther than this threshold if they are LCs:
        if (!is_potential_loop_closure &&
            kf_eucl_dist > params_.max_dist_to_matching)
            continue;

        // Already sent out for checking?
        const auto pair_ids = std::make_pair(
            std::min(kf_id, current_kf_id), std::max(kf_id, current_kf_id));

        {
            std::lock_guard<std::mutex> lck(local_pose_graph_mtx);

            if (state_.local_pose_graph.checked_KF_pairs.count(pair_ids) != 0)
                edge_already_exists = true;
        }

        MRPT_TODO("Factors should have an annotation to know who created them");
        // Also check in the WorldModel if *we* created an edge already between
        // those two KFs:
        if (!edge_already_exists && worldmodel_)
        {
            profiler_.enter("checkForNearbyKFs.wait.worldmodel.locks");

            worldmodel_->entities_lock_for_read();
            worldmodel_->factors_lock_for_read();

            profiler_.leave("checkForNearbyKFs.wait.worldmodel.locks");

            const auto connected = worldmodel_->entity_neighbors(kf_id);
            if (connected.count(current_kf_id) != 0)
            {
                MRPT_LOG_DEBUG_STREAM(
                    "[checkForNearbyKFs] Discarding pair check since a factor "
                    "already exists between #"
                    << kf_id << " <==> #" << current_kf_id);
                edge_already_exists = false;
            }

            worldmodel_->factors_unlock_for_read();
            worldmodel_->entities_unlock_for_read();
        }

        if (!edge_already_exists)
        {
            // Prepare the command to be sent out to the worker thread:
            auto d     = std::make_shared<ICP_Input>();
            d->to_id   = kf_id;
            d->from_id = current_kf_id;

            // Retrieve the point clouds from the Map (WorldModel).
            // This will automatically load them from disk if they were swapped
            // off memory after a long time unused:
            // d->to_pc   = state_.local_pose_graph.pcs[d->to_id];
            // d->from_pc = state_.local_pose_graph.pcs[d->from_id];
            profiler_.enter("checkForNearbyKFs.wait.entities.lockread");

            worldmodel_->entities_lock_for_read();

            profiler_.leave("checkForNearbyKFs.wait.entities.lockread");

            MRPT_TODO(
                "Make a mola-kernel function to make this cleaner and throw "
                "sensible exception errors if something fails");
            {
                ProfilerEntry tle(
                    profiler_, "checkForNearbyKFs.readPCsFromWorldModel");

                d->to_pc = mrpt::ptr_cast<lidar_scan_t>::from(
                    worldmodel_->entity_annotations_by_id(d->to_id)
                        .at(ANNOTATION_NAME_PC_LAYERS)
                        .value());

                d->from_pc = mrpt::ptr_cast<lidar_scan_t>::from(
                    worldmodel_->entity_annotations_by_id(d->from_id)
                        .at(ANNOTATION_NAME_PC_LAYERS)
                        .value());
            }

            worldmodel_->entities_unlock_for_read();

            {
                std::lock_guard<std::mutex> lck(local_pose_graph_mtx);

                d->init_guess_to_wrt_from =
                    state_.local_pose_graph.graph.nodes[kf_id].asTPose();
            }

            // Is this an extra edge for a nearby KF, or a potential loop
            // closure?
            if (!is_potential_loop_closure)
            {
                // Regular, nearby KF-to-KF ICP check:
                d->align_kind = AlignKind::NearbyAlign;
                d->debug_str  = "extra_edge"s;
                d->icp_params = params_.icp_params_with_vel;

                nearby_checks.emplace_back(std::move(d));
            }
            else
            {
                // Attempt to close a loop:
                d->align_kind = AlignKind::LoopClosure;
                d->debug_str  = "loop_closure"s;
                d->icp_params = params_.icp_params_loopclosure;

                loop_closure_checks[kf_eucl_dist] = std::move(d);
            }
        }
    }

    // Actually send the tasks to the worker thread, firstly filtering
    // some of them to reduce the computational cost:
    // Nearby checks: send a maximum of "N"
    const size_t nNearbyChecks    = nearby_checks.size();
    const size_t nearbyCheckDecim = std::max(
        static_cast<size_t>(1U),
        nNearbyChecks / params_.max_nearby_align_checks);
    for (size_t idx = 0; idx < nNearbyChecks; idx += nearbyCheckDecim)
    {
        const auto& d = nearby_checks[idx];
        worker_pool_past_KFs_.enqueue(
            &LidarOdometry3D::doCheckForNonAdjacentKFs, this, d);

        {
            std::lock_guard<std::mutex> lck(local_pose_graph_mtx);

            // Mark as already considered for check:
            state_.local_pose_graph.checked_KF_pairs.insert(std::make_pair(
                std::min(d->to_id, d->from_id),
                std::max(d->to_id, d->from_id)));
        }
    }
    // Loop closures: just send the one with the smallest distance (in theory,
    // it *might* be the easiest one to align...)
    if (!loop_closure_checks.empty())
    {
        const auto& d = loop_closure_checks.begin()->second;

        worker_pool_past_KFs_.enqueue(
            &LidarOdometry3D::doCheckForNonAdjacentKFs, this, d);

        MRPT_LOG_WARN_STREAM(
            "Attempting to close a loop between KFs #" << d->to_id << " <==> #"
                                                       << d->from_id);
        {
            std::lock_guard<std::mutex> lck(local_pose_graph_mtx);
            // Mark as already considered for check:
            state_.local_pose_graph.checked_KF_pairs.insert(std::make_pair(
                std::min(d->to_id, d->from_id),
                std::max(d->to_id, d->from_id)));
        }
    }

    MRPT_END
}

void LidarOdometry3D::doCheckForNonAdjacentKFs(ICP_Input::Ptr d)
{
    try
    {
        ProfilerEntry tleg(profiler_, "doCheckForNonAdjacentKFs");

        // Call ICP:
        ICP_Output icp_out;

        if (d->align_kind != AlignKind::LoopClosure)
        {
            // Regular case:
            ProfilerEntry tle(profiler_, "doCheckForNonAdjacentKFs.run_icp");
            run_one_icp(*d, icp_out);
        }
        else
        {
            // Loop closure:
            ProfilerEntry tle(
                profiler_, "doCheckForNonAdjacentKFs.run_icp_loop_closure");

            // do a small montecarlo sampling and keep the best attempt:
            const double std_xyz = params_.max_dist_to_loop_closure * 0.1;
            const double std_rot = mrpt::DEG2RAD(2.0);

            const auto original_guess = d->init_guess_to_wrt_from;

            mrpt::random::CRandomGenerator rnd;

            for (size_t i = 0; i < params_.loop_closure_montecarlo_samples; i++)
            {
                d->init_guess_to_wrt_from = original_guess;
                d->init_guess_to_wrt_from.x += rnd.drawGaussian1D(0, std_xyz);
                d->init_guess_to_wrt_from.y += rnd.drawGaussian1D(0, std_xyz);
                d->init_guess_to_wrt_from.z += rnd.drawGaussian1D(0, std_xyz);
                d->init_guess_to_wrt_from.yaw += rnd.drawGaussian1D(0, std_rot);

                ICP_Output this_icp_out;
                run_one_icp(*d, this_icp_out);
                if (this_icp_out.goodness > icp_out.goodness)
                    icp_out = this_icp_out;
            }
        }

        const mrpt::poses::CPose3D rel_pose =
            icp_out.found_pose_to_wrt_from.getMeanVal();
        const double icp_goodness = icp_out.goodness;

        // Accept the new edge?
        const mrpt::poses::CPose3D init_guess(d->init_guess_to_wrt_from);
        const double pos_correction = (rel_pose - init_guess).norm();
        const double correction_percent =
            pos_correction / (init_guess.norm() + 0.01);

        MRPT_LOG_DEBUG_STREAM(
            "[doCheckForNonAdjacentKFs] Checking KFs: #"
            << d->from_id << " ==> #" << d->to_id
            << " init_guess: " << d->init_guess_to_wrt_from.asString() << "\n"
            << mrpt::format("ICP goodness=%.03f\n", icp_out.goodness)
            << "ICP rel_pose=" << rel_pose.asString() << " init_guess was "
            << init_guess.asString() << " (changes " << 100 * correction_percent
            << "%)");

        const double goodness_thres =
            (d->align_kind == AlignKind::LoopClosure
                 ? params_.min_icp_goodness_lc
                 : params_.min_icp_goodness);

        if (icp_goodness > goodness_thres &&
            (correction_percent < 0.2 ||
             d->align_kind == AlignKind::LoopClosure))
        {
            std::future<BackEndBase::AddFactor_Output> factor_out_fut;
            mola::FactorRelativePose3                  fPose3(
                d->from_id, d->to_id, rel_pose.asTPose());

            mola::Factor f = std::move(fPose3);
            factor_out_fut = slam_backend_->addFactor(f);

            // Wait until it's executed:
            auto factor_out = factor_out_fut.get();
            ASSERT_(factor_out.success);
            ASSERT_(
                factor_out.new_factor_id &&
                factor_out.new_factor_id != mola::INVALID_FID);

            // Append to local graph as well::
            {
                std::lock_guard<std::mutex> lck(local_pose_graph_mtx);
                state_.local_pose_graph.graph.insertEdgeAtEnd(
                    d->from_id, d->to_id, rel_pose);
            }

            MRPT_LOG_DEBUG_STREAM(
                "New FactorRelativePose3: #"
                << d->from_id << " <=> #" << d->to_id
                << ". rel_pose=" << rel_pose.asString());
        }
    }
    catch (const std::exception& e)
    {
        MRPT_LOG_ERROR_STREAM("Exception:\n" << mrpt::exception_to_str(e));
    }
}

void LidarOdometry3D::run_one_icp(const ICP_Input& in, ICP_Output& out)
{
    using namespace std::string_literals;

    MRPT_START

    // A reference number for each ICP run, just to help identifying log
    // messages from one single run when running in parallel threads.
    static unsigned int run_id = 0;
    static std::mutex   run_id_mx;
    run_id_mx.lock();
    const auto this_run_id = run_id++;
    run_id_mx.unlock();

    {
        ProfilerEntry tle(profiler_, "run_one_icp");

        ASSERT_(!in.icp_params.empty());

        const p2p2::PointsPlanesICP::pointcloud_t& pcs_from = in.from_pc->pc;
        const p2p2::PointsPlanesICP::pointcloud_t& pcs_to   = in.to_pc->pc;

        mrpt::math::TPose3D current_solution = in.init_guess_to_wrt_from;

        for (unsigned int stage = 0; stage < in.icp_params.size(); stage++)
        {
            p2p2::Parameters icp_params = in.icp_params[stage];
            icp_params.max_corresponding_points =
                params_.max_correspondences_per_layer;

            MRPT_TODO("explain/refactor");
            icp_params.ignore_point_layers.clear();
            icp_params.ignore_point_layers.insert("raw"s);
            if (stage != (in.icp_params.size() - 1))
                icp_params.ignore_point_layers.insert("resampled_planes"s);

            p2p2::Results icp_result;
            p2p2::PointsPlanesICP::align(
                pcs_from, pcs_to, current_solution, icp_params, icp_result);

            if (icp_result.goodness > 0)
            {
                // Keep as init value for next stage:
                current_solution = icp_result.optimal_tf.mean.asTPose();
            }

            out.found_pose_to_wrt_from = icp_result.optimal_tf;
            out.goodness               = icp_result.goodness;

            MRPT_LOG_DEBUG_FMT(
                "[ICP run #%u] stage #%u (kind=%u): goodness=%.03f iters=%u "
                "rel_pose=%s termReason=%u",
                this_run_id, stage, static_cast<unsigned int>(in.align_kind),
                out.goodness, static_cast<unsigned int>(icp_result.nIterations),
                out.found_pose_to_wrt_from.getMeanVal().asString().c_str(),
                static_cast<unsigned int>(icp_result.terminationReason));
        }

        // Check quality of match:
        MRPT_TODO("Impl. finite differences based Hessian check");
    }

    // -------------------------------------------------
    MRPT_TODO("Move this to its own method");

    // Save debug files for debugging ICP quality
    bool gen_debug = (in.align_kind == AlignKind::LidarOdometry &&
                      params_.debug_save_lidar_odometry) ||
                     (in.align_kind == AlignKind::NearbyAlign &&
                      params_.debug_save_extra_edges) ||
                     (in.align_kind == AlignKind::LoopClosure &&
                      params_.debug_save_loop_closures);

    if (gen_debug)
    {
        const auto num_pc_layers = in.from_pc->pc.point_layers.size();

        for (unsigned int l = 0; l < num_pc_layers; l++)
        {
            auto fil_name_prefix = mrpt::system::fileNameStripInvalidChars(
                getModuleInstanceName() +
                mrpt::format(
                    "_debug_ICP_%s_%05u_layer%02u", in.debug_str.c_str(),
                    static_cast<unsigned int>(this_run_id), l));

            // Init:
            mrpt::opengl::COpenGLScene scene;

            auto it_from = in.from_pc->pc.point_layers.begin();
            std::advance(it_from, l);
            auto it_to = in.to_pc->pc.point_layers.begin();
            std::advance(it_to, l);

            scene.insert(
                mrpt::opengl::stock_objects::CornerXYZSimple(2.0f, 4.0f));
            auto gl_from = mrpt::opengl::CSetOfObjects::Create();
            it_from->second->renderOptions.color =
                mrpt::img::TColorf(.0f, .0f, 1.0f);
            it_from->second->getAs3DObject(gl_from);
            gl_from->setName("KF_from"s);
            gl_from->enableShowName();
            scene.insert(gl_from);

            auto gl_to = mrpt::opengl::CSetOfObjects::Create();
            gl_to->insert(
                mrpt::opengl::stock_objects::CornerXYZSimple(1.0f, 2.0f));
            it_to->second->renderOptions.color =
                mrpt::img::TColorf(1.0f, .0f, .0f);
            it_to->second->getAs3DObject(gl_to);
            gl_to->setName("KF_to"s);
            gl_to->enableShowName();
            scene.insert(gl_to);

            auto gl_info  = mrpt::opengl::CText::Create(),
                 gl_info2 = mrpt::opengl::CText::Create();
            gl_info->setLocation(0., 0., 5.);
            gl_info2->setLocation(0., 0., 4.8);
            scene.insert(gl_info);
            scene.insert(gl_info2);

            {
                std::ostringstream ss;
                ss << "to_ID     = " << in.to_id
                   << " from_ID   = " << in.from_id << " | " << in.debug_str;
                gl_info->setString(ss.str());
            }
            {
                std::ostringstream ss;
                ss << "init_pose = " << in.init_guess_to_wrt_from.asString();
                gl_info2->setString(ss.str());
            }

            const auto fil_name_init = fil_name_prefix + "_0init.3Dscene"s;
            if (scene.saveToFile(fil_name_init))
                MRPT_LOG_DEBUG_STREAM(
                    "Wrote debug init ICP scene to: " << fil_name_init);
            else
                MRPT_LOG_ERROR_STREAM(
                    "Error saving init ICP scene to :" << fil_name_init);

            // Final:
            const auto final_pose = out.found_pose_to_wrt_from.getMeanVal();
            gl_to->setPose(final_pose);

            {
                std::ostringstream ss;
                ss << "to_ID     = " << in.to_id
                   << " from_ID   = " << in.from_id;
                gl_info->setString(ss.str());
            }
            {
                std::ostringstream ss;
                ss << " final_pose = " << final_pose.asString()
                   << " goodness: " << out.goodness * 100.0;

                gl_info2->setString(ss.str());
            }

            const auto fil_name_final = fil_name_prefix + "_1final.3Dscene"s;
            if (scene.saveToFile(fil_name_final))
                MRPT_LOG_DEBUG_STREAM(
                    "Wrote debug final ICP scene to: " << fil_name_final);
            else
                MRPT_LOG_ERROR_STREAM(
                    "Error saving final ICP scene to :" << fil_name_final);

            // Also: save as Rawlog for ICP debugging in RawLogViewer app:
            {
#if 0
                mrpt::obs::CRawlog rawlog;
                {
                    auto pc = mrpt::obs::CObservationPointCloud::Create();
                    pc->pointcloud  = in.from_pc.sampled;
                    pc->sensorLabel = "from_sampled";
                    rawlog.addObservationMemoryReference(pc);
                }
                {
                    auto pc = mrpt::obs::CObservationPointCloud::Create();
                    pc->pointcloud  = in.to_pc.sampled;
                    pc->sensorLabel = "to_sampled";
                    rawlog.addObservationMemoryReference(pc);
                }
            {
                auto pc         = mrpt::obs::CObservationPointCloud::Create();
                pc->pointcloud  = in.from_pc.original;
                pc->sensorLabel = "from_original";
                rawlog.addObservationMemoryReference(pc);
            }
            {
                auto pc         = mrpt::obs::CObservationPointCloud::Create();
                pc->pointcloud  = in.to_pc.original;
                pc->sensorLabel = "to_original";
                rawlog.addObservationMemoryReference(pc);
            }
            {
                mrpt::config::CConfigFileMemory cfg;
                in.icp_params.saveToConfigFile(cfg, "ICP");

                auto comm  = mrpt::obs::CObservationComment::Create();
                comm->text = cfg.getContent();
                rawlog.addObservationMemoryReference(comm);
            }

            const auto fil_name_rawlog = fil_name_prefix + ".rawlog"s;
            if (rawlog.saveToRawLogFile(fil_name_rawlog))
                MRPT_LOG_DEBUG_STREAM(
                    "Wrote ICP debug rawlog: " << fil_name_rawlog);
#endif
            }
        }
    }

    MRPT_END
}

template <typename VEC>
static void pointcloud_centroid_covariance(
    const VEC& xs, const VEC& ys, const VEC& zs,
    const p2p2::PointCloudToVoxelGrid::voxel_t& vxl_pts,
    mrpt::math::TPoint3Df& mean, Eigen::Matrix3f& cov)
{
    mean              = mrpt::math::TPoint3Df(0, 0, 0);
    const float inv_n = (1.0f / vxl_pts.indices.size());
    for (size_t i = 0; i < vxl_pts.indices.size(); i++)
    {
        const auto pt_idx = vxl_pts.indices[i];
        mean.x += xs[pt_idx];
        mean.y += ys[pt_idx];
        mean.z += zs[pt_idx];
    }
    mean.x *= inv_n;
    mean.y *= inv_n;
    mean.z *= inv_n;

    cov.setZero();
    for (size_t i = 0; i < vxl_pts.indices.size(); i++)
    {
        const auto                  pt_idx = vxl_pts.indices[i];
        const mrpt::math::TPoint3Df a(
            xs[pt_idx] - mean.x, ys[pt_idx] - mean.y, zs[pt_idx] - mean.z);
        cov(0, 0) += a.x * a.x;
        cov(1, 0) += a.x * a.y;
        cov(2, 0) += a.x * a.z;
        cov(1, 1) += a.y * a.y;
        cov(2, 1) += a.y * a.z;
        cov(2, 2) += a.z * a.z;
    }
    cov *= inv_n;
}

LidarOdometry3D::lidar_scan_t LidarOdometry3D::filterPointCloud(
    const mrpt::maps::CPointsMap& pc)
{
    MRPT_START

    // The result:
    lidar_scan_t scan;

    auto& pc_edges            = scan.pc.point_layers["edges"];
    auto& pc_plane_centroids  = scan.pc.point_layers["plane_centroids"];
    auto& pc_color_bright     = scan.pc.point_layers["color_bright"];
    auto& pc_resampled_planes = scan.pc.point_layers["resampled_planes"];
    auto& planes              = scan.pc.planes;

    auto pc_xyzi = dynamic_cast<const mrpt::maps::CPointsMapXYZI*>(&pc);

    pc_edges            = mrpt::maps::CSimplePointsMap::Create();
    pc_plane_centroids  = mrpt::maps::CSimplePointsMap::Create();
    pc_color_bright     = mrpt::maps::CSimplePointsMap::Create();
    pc_resampled_planes = mrpt::maps::CSimplePointsMap::Create();

    pc_edges->reserve(pc.size() / 10);
    planes.reserve(pc.size() / 1000);
    pc_plane_centroids->reserve(pc.size() / 1000);
    pc_resampled_planes->reserve(pc.size() / 200);

    state_.filter_grid4edges.clear();
    state_.filter_grid4edges.processPointCloud(pc);

    state_.filter_grid4planes.clear();
    state_.filter_grid4planes.processPointCloud(pc);

    const auto& xs = pc.getPointsBufferRef_x();
    const auto& ys = pc.getPointsBufferRef_y();
    const auto& zs = pc.getPointsBufferRef_z();

    std::size_t nEdgeVoxels = 0, nPlaneVoxels = 0;
    for (const auto vxl_idx : state_.filter_grid4edges.used_voxel_indices)
    {
        const auto& vxl_pts =
            state_.filter_grid4edges.pts_voxels.cellByIndex(vxl_idx);

        if (vxl_pts->indices.size() <
            params_.voxel_filter4edges_min_point_count)
            continue;

        // Analyze the voxel contents:
        mrpt::math::TPoint3Df mean;
        Eigen::Matrix3f       cov;
        pointcloud_centroid_covariance(xs, ys, zs, *vxl_pts, mean, cov);

        // This only looks at the lower-triangular part of the cov
        // matrix (which is(was->fixed via PR) wrong in loam_velodyne!)
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(cov);

        const Eigen::Vector3f eig_vals = esolver.eigenvalues();
        const float e0 = eig_vals[0], e1 = eig_vals[1], e2 = eig_vals[2];

        if (e2 > params_.voxel_filter4edges_min_e2_e1 * e1 &&
            e1 < params_.voxel_filter4edges_max_e1_e0 * e0)
        {
            // Classified as EDGE
            // ------------------------
            nEdgeVoxels++;
            for (size_t i = 0; i < vxl_pts->indices.size(); i++)
            {
                const auto pt_idx = vxl_pts->indices[i];
                const auto ptx = xs[pt_idx], pty = ys[pt_idx], ptz = zs[pt_idx];

                if ((i % params_.voxel_filter4edges_decimation) == 0)
                    pc_edges->insertPointFast(ptx, pty, ptz);

                if (pc_xyzi)
                {
                    MRPT_TODO("Dynamic thresholds?");
                    const float pt_int =
                        pc_xyzi->getPointIntensity_fast(pt_idx);
                    if (pt_int > 0.75f)
                        pc_color_bright->insertPointFast(ptx, pty, ptz);
                }
            }
        }
    }  // end for each voxel

    // Planes:
    for (const auto vxl_idx : state_.filter_grid4planes.used_voxel_indices)
    {
        const auto& vxl_pts =
            state_.filter_grid4planes.pts_voxels.cellByIndex(vxl_idx);

        if (vxl_pts->indices.size() <
            params_.voxel_filter4planes_min_point_count)
            continue;

        // Analyze the voxel contents:
        mrpt::math::TPoint3Df mean;
        Eigen::Matrix3f       cov;
        pointcloud_centroid_covariance(xs, ys, zs, *vxl_pts, mean, cov);

        // This only looks at the lower-triangular part of the cov
        // matrix (which is(was->fixed via PR) wrong in loam_velodyne!)
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(cov);

        const Eigen::Vector3f eig_vals = esolver.eigenvalues();
        // The eigenvalues are sorted in increasing order.
        const float e0 = eig_vals[0], e1 = eig_vals[1], e2 = eig_vals[2];

        const Eigen::Vector3f ev0 = esolver.eigenvectors().col(0);

        if (e1 > params_.voxel_filter4planes_min_e1_e0 * e0 &&
            e2 > params_.voxel_filter4planes_min_e2_e0 * e0)
        {
            // Classified as PLANE
            // ------------------------
            nPlaneVoxels++;

            // Define a plane from its centroid + a normal:
            const auto pl_c = mrpt::math::TPoint3D(mean);

            // Normal = largest eigenvector:
            auto pl_n = mrpt::math::TVector3D(ev0.x(), ev0.y(), ev0.z());

            // Normal direction criterion: make it to face towards the vehicle.
            // We can use the dot product to find it out, since pointclouds are
            // given in vehicle-frame coordinates.
            {
                // Unit vector: vehicle -> plane centroid:
                ASSERT_ABOVE_(pl_c.norm(), 1e-3);
                const auto u = pl_c * (1.0 / pl_c.norm());
                const auto dot_prod =
                    mrpt::math::dotProduct<3, double>(u, pl_n);

                // It should be <0 if the normal is pointing to the vehicle.
                // Otherwise, reverse the normal.
                if (dot_prod > 0) pl_n = -pl_n;
            }

            // Add plane & centroid:
            const auto pl = mrpt::math::TPlane3D(pl_c, pl_n);
            planes.emplace_back(pl, pl_c);

            // Also: add the centroid to this special layer:
            pc_plane_centroids->insertPointFast(pl_c.x, pl_c.y, pl_c.z);

            // Also, add a set of resampled points to a special layer:
            // -----------------------------------------------------------
            if (pl_c.norm() > 20.0)
            {
                MRPT_TODO("params!");

                const Eigen::Vector3f ev1 = esolver.eigenvectors().col(1),
                                      ev2 = esolver.eigenvectors().col(2);
                const int nP              = 2;
                for (int ix = -nP; ix <= nP; ix++)
                    for (int iy = -nP; iy <= nP; iy++)
                    {
                        const float K =
                            params_.voxel_filter4planes_resolution / nP;
                        pc_resampled_planes->insertPointFast(
                            pl_c.x + ix * K * ev1.x() + iy * K * ev2.x(),
                            pl_c.y + ix * K * ev1.y() + iy * K * ev2.y(),
                            pl_c.z + ix * K * ev1.z() + iy * K * ev2.z());
                    }
            }
        }

    }  // end for each voxel

    // Mark all pcs as "modified" (to rebuild the kd-trees, etc.), since we used
    // the "fast" insert methods above:
    pc_color_bright->mark_as_modified();
    pc_edges->mark_as_modified();
    pc_plane_centroids->mark_as_modified();
    pc_resampled_planes->mark_as_modified();

    MRPT_LOG_DEBUG_STREAM(
        "[VoxelGridFilter] Voxel counts:\n"
        << " edges=" << nEdgeVoxels << " planes=" << nPlaneVoxels);

    return scan;

    MRPT_END
}
