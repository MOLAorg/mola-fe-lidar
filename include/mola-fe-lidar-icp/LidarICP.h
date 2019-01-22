/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   LidarICP.h
 * @brief  Simple SLAM FrontEnd for point-cloud sensors via ICP registration
 * @author Jose Luis Blanco Claraco
 * @date   Dec 17, 2018
 */
#pragma once

#include <mola-kernel/FrontEndBase.h>
#include <mola-kernel/WorkerThreadsPool.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/slam/CICP.h>
#include <mutex>
#include "CPointCloudVoxelGrid.h"
#include "MultiCloudICP.h"

namespace mola
{
/** A front-end for Lidar/point-cloud odometry & SLAM.
 *
 * \ingroup mola_fe_lidar_icp_grp */
class LidarICP : public FrontEndBase
{
   public:
    LidarICP();
    virtual ~LidarICP() override = default;

    // See docs in base class
    void initialize(const std::string& cfg_block) override;
    void spinOnce() override;
    void onNewObservation(CObservation::Ptr& o) override;

    /** Re-initializes the front-end */
    void reset();

    struct Parameters
    {
        /** Minimum time (seconds) between scans for being attempted to be
         * aligned. Scans faster than this rate will be just silently ignored.
         */
        double min_time_between_scans{0.2};

        /** Minimum Euclidean distance (x,y,z) between keyframes inserted into
         * the map [meters]. */
        double min_dist_xyz_between_keyframes{1.0};

        /** Minimum rotation (in 3D space, yaw, pitch,roll, altogether) between
         * keyframes inserted into
         * the map [rad here, degrees in the yaml file]. */
        double min_rotation_between_keyframes{mrpt::DEG2RAD(30.0)};

        /** Minimum ICP "goodness" (in the range [0,1]) for a new KeyFrame to be
         * accepted during regular lidar odometry & mapping */
        double min_icp_goodness{0.4};

        /** Minimum ICP quality for a loop closure to be accepted */
        double min_icp_goodness_lc{0.6};

        /** If !=0, decimate point clouds so they do not have more than this
         * number of points */
        unsigned int decimate_to_point_count{500};

        /** Size of the voxel filter [meters] */
        double voxel_filter_resolution{.5};
        double voxel_filter_max_e2_e0{30.}, voxel_filter_max_e1_e0{30.};

        /** Distance range to check for additional SE(3) edges */
        double min_dist_to_matching{6.0};
        double max_dist_to_matching{12.0};

        unsigned int max_KFs_local_graph{1000};

        /** ICP parameters for the case of having, or not, a good velocity model
         * that works a good prior. Each entry in the vector is an "ICP stage",
         * to be run as a sequence of coarser to finer detail */
        std::vector<MultiCloudICP::Parameters> icp_params_with_vel,
            icp_params_without_vel, icp_params_loopclosure;

        bool debug_save_lidar_odometry{false};
        bool debug_save_extra_edges{false};
        bool debug_save_loop_closures{false};
    };

    /** Algorithm parameters */
    Parameters params_;

    using topological_dist_t = std::size_t;

    constexpr static topological_dist_t MIN_DIST_TO_CONSIDER_LOOP_CLOSURE = 10;

   private:
    /** The worker thread pool with 1 thread for processing incomming scans */
    mola::WorkerThreadsPool worker_pool_{1};

    /** Worker thread to align a new KF against past KFs:*/
    mola::WorkerThreadsPool worker_pool_past_KFs_{3};

    struct pointclouds_t
    {
        /** Different "layers" in which a point cloud is decomposed.
         * For example: "edges", "planes", etc.
         */
        std::map<std::string, mrpt::maps::CPointsMap::Ptr> layers;

        inline bool hasLayer(const std::string& s) const
        {
            return layers.find(s) != layers.end();
        }
    };

    /** All variables that hold the algorithm state */
    struct MethodState
    {
        mrpt::Clock::time_point last_obs_tim{};
        pointclouds_t           last_points{};
        mrpt::math::TTwist3D    last_iter_twist;
        bool                    last_iter_twist_is_good{false};
        id_t                    last_kf{mola::INVALID_ID};
        mrpt::poses::CPose3D    accum_since_last_kf{};
        CPointCloudVoxelGrid    filter_grid;

        // An auxiliary (local) pose-graph to use Dijkstra and find guesses
        // for ICP against nearby past KFs:
        struct LocalPoseGraph
        {
            mrpt::graphs::CNetworkOfPoses3D                graph;
            std::map<mrpt::graphs::TNodeID, pointclouds_t> pcs;
            /** Pairs of KFs that have been already checked for loop closure */
            std::set<std::pair<id_t, id_t>> checked_KF_pairs;
        };

        LocalPoseGraph local_pose_graph;
    };

    MethodState     state_;
    WorldModel::Ptr worldmodel_;

    /** Here happens the actual processing, invoked from the worker thread pool
     * for each incomming observation */
    void doProcessNewObservation(CObservation::Ptr& o);
    void checkForNearbyKFs();

    enum class AlignKind : uint8_t
    {
        LidarOdometry,
        NearbyAlign,
        LoopClosure
    };

    struct ICP_Input
    {
        using Ptr = std::shared_ptr<ICP_Input>;

        AlignKind           align_kind{AlignKind::LidarOdometry};
        id_t                to_id{mola::INVALID_ID};
        id_t                from_id{mola::INVALID_ID};
        pointclouds_t       to_pc, from_pc;
        mrpt::math::TPose3D init_guess_to_wrt_from;

        std::vector<MultiCloudICP::Parameters> icp_params;

        /** used to identity where does this request come from */
        std::string debug_str;
    };
    struct ICP_Output
    {
        double                          goodness{.0};
        mrpt::poses::CPose3DPDFGaussian found_pose_to_wrt_from;
    };
    void run_one_icp(const ICP_Input& in, ICP_Output& out);

    /** Invoked from doProcessNewObservation() whenever a new KF is created,
     * to check for additional edges apart of the "odometry edge", to increase
     * the quality of the estimation by increasing the pose-graph density.
     */
    void doCheckForNonAdjacentKFs(ICP_Input::Ptr d);

    void filterPointCloud(pointclouds_t& pcs);

    std::mutex local_pose_graph_mtx;

    // Debug aux variables:
    std::atomic<unsigned int> debug_dump_icp_file_counter{0};
};

}  // namespace mola
