#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <numeric>
#include <algorithm>
#include <memory>

#include "include/utility.h" // myTf, down_sampling_voxel, TicToc, color macros,...
#include "include/STDesc.h"  // STDescManager, STDesc
#include <boost/function.hpp>
#include <boost/bind.hpp>

using namespace Eigen;
using namespace pcl;
using namespace Util;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "relocalization_std_online_main");
    ros::NodeHandle nh;

    // ==============================
    // 1) Read all params (inline in main)
    // ==============================
    ConfigSetting config_setting;
    read_parameters(nh, config_setting);

    // extra node params
    std::string descriptor_path;
    std::string map_frame = "map";
    std::string base_frame = "body";
    bool apply_odom_to_cloud = false;
    std::string cloud_topic = "/livox/lidar_ouster";

    nh.param<std::string>("descriptor_file_path", descriptor_path, std::string(""));
    nh.param<std::string>("map_frame", map_frame, map_frame);
    nh.param<std::string>("base_frame", base_frame, base_frame);
    nh.param<bool>("apply_odom_to_cloud", apply_odom_to_cloud, apply_odom_to_cloud);
    nh.param<std::string>("cloud_topic", cloud_topic, cloud_topic);

    // ==============================
    // 2) Create manager in main
    // ==============================
    STDescManager *std_manager = new STDescManager(config_setting);

    // Load DB
    std_manager->loadExistingSTD(descriptor_path);
    ROS_INFO(KGRN "STD database loaded from: %s" RESET, descriptor_path.c_str());

    // ==============================
    // 3) Publishers
    // ==============================
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/relocalization_pose", 10);

    // ==============================
    // 4) Stats (local in main)
    // ==============================
    size_t frame_count = 0, keyframe_count = 0;
    int loop_count = 0;
    std::vector<double> desc_times_ms, query_times_ms;

    /* ============================
     *  Functor callback (Boost-friendly)
     * ============================ */
    struct RelocCB
    {
        STDescManager *std_manager;
        const ConfigSetting *cfg;
        ros::Publisher *pub_pose;
        bool apply_odom_to_cloud; // giờ luôn false, nhưng cứ để nếu sau này muốn dùng lại
        std::string map_frame;

        size_t *frame_count;
        size_t *keyframe_count;
        int *loop_count;
        std::vector<double> *desc_times_ms;
        std::vector<double> *query_times_ms;

        typedef void result_type;

        void operator()(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) const
        {
            (*frame_count)++;

            // Convert cloud
            CloudXYZIPtr current_cloud(new CloudXYZI());
            pcl::fromROSMsg(*cloud_msg, *current_cloud);

            // Không còn odom nên bỏ đoạn W_T_B, transform theo odom
            // if (apply_odom_to_cloud) { ... }  // bỏ hẳn

            // Downsample nếu muốn
            // down_sampling_voxel(*current_cloud, cfg->ds_size_);

            // Keyframe gating
            if (((*frame_count) - 1) % cfg->sub_frame_num_ != 0)
                return;
            (*keyframe_count)++;

            // 1) Descriptor extraction (one-time)
            TicToc tt_desc;
            std::vector<STDesc> stds_vec;
            std_manager->GenerateSTDescsOneTime(current_cloud, stds_vec);
            desc_times_ms->push_back(tt_desc.Toc());

            // 2) Loop search
            TicToc tt_query;
            std::pair<int, double> search_result(-1, 0.0);
            std::pair<Vector3d, Matrix3d> loop_tf({Vector3d::Zero(), Matrix3d::Identity()});
            std::vector<std::pair<STDesc, STDesc>> loop_pairs;

            std_manager->SearchLoop(stds_vec, search_result, loop_tf, loop_pairs);
            query_times_ms->push_back(tt_query.Toc());

            if (search_result.first >= 0)
            {
                ROS_INFO(KGRN "[Loop] kf=%zu db=%d score=%.5f" RESET,
                         (*keyframe_count) - 1, search_result.first, search_result.second);

                std::pair<Vector3d, Matrix3d> refined = loop_tf;
                std_manager->PlaneGeomrtricIcp(std_manager->current_plane_cloud_,
                                               std_manager->plane_cloud_vec_[search_result.first],
                                               refined);

                myTf tf_W_B_est(refined.second, refined.first);
                const Eigen::Vector3d &t = tf_W_B_est.pos;
                const Eigen::Quaterniond qe = tf_W_B_est.rot;

                double yaw = qe.toRotationMatrix().eulerAngles(2, 1, 0)(0);
                if (!std::isfinite(yaw))
                    yaw = 0.0;

                Eigen::Quaterniond qe_yaw(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

                geometry_msgs::PoseStamped msg;
                // dùng stamp của cloud
                msg.header.stamp = cloud_msg->header.stamp;
                msg.header.frame_id = map_frame;

                msg.pose.position.x = t.x();
                msg.pose.position.y = t.y();
                msg.pose.position.z = t.z();

                msg.pose.orientation.w = qe_yaw.w();
                msg.pose.orientation.x = qe_yaw.x();
                msg.pose.orientation.y = qe_yaw.y();
                msg.pose.orientation.z = qe_yaw.z();

                pub_pose->publish(msg);
                (*loop_count)++;
            }
        }
    };

    RelocCB cb{
        /* std_manager        */ std_manager,
        /* cfg                */ &config_setting,
        /* pub_pose           */ &pub_pose,
        /* apply_odom_to_cloud*/ apply_odom_to_cloud,
        /* map_frame          */ map_frame,
        /* frame_count        */ &frame_count,
        /* keyframe_count     */ &keyframe_count,
        /* loop_count         */ &loop_count,
        /* desc_times_ms      */ &desc_times_ms,
        /* query_times_ms     */ &query_times_ms};

    // ==============================
    // 5) Subscriber cloud ONLY
    // ==============================
    ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>(
        cloud_topic, 50,
        boost::bind(&RelocCB::operator(), &cb, _1));

    // ==============================
    // 6) Spin & print stats on shutdown
    // ==============================
    ros::spin();

    auto mean = [](const std::vector<double> &v) -> double
    {
        if (v.empty())
            return 0.0;
        return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    };

    ROS_INFO("Total frames=%zu, keyframes=%zu, loop triggers=%d",
             frame_count, keyframe_count, loop_count);
    ROS_INFO("Mean times: desc=%.3f ms, query=%.3f ms",
             mean(desc_times_ms), mean(query_times_ms));

    delete std_manager;
    return 0;
}
