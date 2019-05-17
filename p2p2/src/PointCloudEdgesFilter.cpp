/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <p2p2/PointCloudEdgesFilter.h>

using namespace p2p2;

void PointCloudEdgesFilter::processPointCloud(
    const mrpt::maps::CPointsMap& in, mrpt::maps::CPointsMap& out)
{
    MRPT_START

    out.clear();

    ASSERT_ABOVE_(params_.search_window, 0);

    const auto& xs   = in.getPointsBufferRef_x();
    const auto& ys   = in.getPointsBufferRef_y();
    const auto& zs   = in.getPointsBufferRef_z();
    const auto  npts = xs.size();
    const auto  w    = 4;  // params_.search_window;

    if (npts < 2 * w) return;  // nothing to do!

    out.reserve(npts / 100);

    const auto i_end = npts - 2 * (w + 1);

    for (std::size_t i = 0; i < i_end; i += w / 2)
    {
        //
        mrpt::math::TVector3D sum_l(0, 0, 0), sum_r(0, 0, 0);
        for (int k = 1; k <= w; k++)
        {
            sum_r += mrpt::math::TVector3D(
                xs[i + w + k] - xs[i + w], ys[i + w + k] - ys[i + w],
                zs[i + w + k] - zs[i + w]);
            sum_l += mrpt::math::TVector3D(
                xs[i + w - k] - xs[i + w], ys[i + w - k] - ys[i + w],
                zs[i + w - k] - zs[i + w]);
        }

        sum_r *= 1.0 / w;
        sum_l *= 1.0 / w;

        double corner_score =
            (sum_l + sum_r).norm() / (sum_l.norm() + sum_r.norm());

#if 0
        static FILE* f = fopen("a.txt", "wt");
        fprintf(f, "%f\n", corner_score);
#endif
        if (corner_score > 0.9)
        {
            for (int k = 0; k < 2 * w + 1; k++)
                out.insertPointFast(xs[i + k], ys[i + k], zs[i + k]);
        }
    }

    out.mark_as_modified();  // since we used the "fast" insert method above

    //
    MRPT_END
}
