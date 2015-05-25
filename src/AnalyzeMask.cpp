#include <ros/ros.h>

// point cloud stuff
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <Eigen/Dense>
#include <limits>

// image stuff
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

// local stuff
#include <apc/MatInfo.h>
#include <apc/Recognized.h>
#include <apc/Recognition.h>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud;

class AnalyzeMask {
    public: 
        AnalyzeMask();
        void receive_mask_info(apc::MatInfo);
        void receive_scene(const sensor_msgs::PointCloud2&);
        void receive_mask(const sensor_msgs::ImageConstPtr&);
    private:
        cv::Mat mask_3d;
        cv::Mat mask_2d;
        Cloud scene;
        std_msgs::Header scene_header;

        int job_number;

        int ndet;
        std::vector<int> det2cat;
        std::vector<float> det2score;
        std::vector<float> centroids_x;
        std::vector<float> centroids_y;
        std::vector<float> angles_ma;
        std::vector<float> angles_mi;

        ros::Subscriber mask_info_sub;
        ros::Subscriber scene_sub;
        image_transport::Subscriber mask_sub;
        ros::Publisher rec_pub;
        ros::Publisher glue_pub;

        ros::NodeHandle nh;
        image_transport::ImageTransport itnh;
        //tf::TransformListener transform;
        Eigen::Affine3f transform;

        std::vector<Cloud> apply_mask();
        std::vector<std::vector<float> > analyze_3d(std::vector<Cloud>);
        cv::Mat remap(cv::Mat);
};

AnalyzeMask::AnalyzeMask() : itnh(nh) {
    mask_3d = cv::Mat::zeros(480, 640, CV_8UC1);
    mask_2d = cv::Mat::zeros(400, 640, CV_8UC1);
 
    mask_info_sub = nh.subscribe("/apc/recognition_information", 1, 
                                 &AnalyzeMask::receive_mask_info, this);
    //scene_sub = nh.subscribe("/apc/recognition_pcl", 1, 
    //                         &AnalyzeMask::receive_scene, this);
    scene_sub = nh.subscribe("/camera/depth/points", 1, 
                             &AnalyzeMask::receive_scene, this);
    mask_sub = itnh.subscribe("/apc/recognition_mask_image", 1, 
                              &AnalyzeMask::receive_mask, this);
    rec_pub = nh.advertise<apc::Recognition>("/apc/recognition", 1);
    glue_pub = nh.advertise<sensor_msgs::PointCloud2>("test", 1);

    transform = Eigen::Affine3f::Identity();
    transform.translation() << 0.15, 0.09, 1.08;
    transform.rotate(Eigen::AngleAxisf(0.68, Eigen::Vector3f::UnitY()));
}

// Receive flag for 2d/3d, point cloud if necessary, and two vectors
void AnalyzeMask::receive_mask_info(apc::MatInfo mask_msg) {
    ROS_INFO("> received mask info");
    job_number = mask_msg.job_number;
    det2cat = mask_msg.categories;
    det2score = mask_msg.scores;
    ndet = mask_msg.number_of_items;
    centroids_x = mask_msg.centroids_x;
    centroids_y = mask_msg.centroids_y;
    angles_ma = mask_msg.angles_ma;
    angles_mi = mask_msg.angles_mi;

    if (mask_msg.is3d) {
        // pull out info
        std::vector<Cloud> clusters = apply_mask();
        std::vector<std::vector<float> > data_3d = analyze_3d(clusters);

        // publish
        std::vector<apc::Recognized> objects;
        for (int i = 1; i <= ndet; i++) {
            if (data_3d[i].size() == 0) { continue; }
            if ((int)centroids_x[i] == -1) { continue; }
            if (det2cat[i] > 30) { continue; }
            apc::Recognized rec;
            rec.is3d = mask_msg.is3d;
            rec.job_number = job_number;
            rec.obj_id = det2cat[i];
            rec.confidence = det2score[i];
            rec.centroid_x = data_3d[i][0];
            rec.centroid_y = data_3d[i][1];
            rec.centroid_z = data_3d[i][2];
            rec.angle_ma = angles_ma[i];
            rec.angle_mi = angles_mi[i];
            objects.push_back(rec);
            ROS_INFO("> found %d at (%.02f, %.02f, %.02f)",
                     det2cat[i], data_3d[i][0], 
                     data_3d[i][1], data_3d[i][2]);
        }
        apc::Recognition rec_msg;
        rec_msg.recognitions = objects;
        rec_pub.publish(rec_msg);

        sensor_msgs::PointCloud2::Ptr testmsg (new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*clusters[58], *testmsg);
        //pcl::toROSMsg(*scene, *testmsg);
        testmsg->header = scene_header;
        glue_pub.publish(testmsg);
    } else {
        std::vector<apc::Recognized> objects;
        for (int i = 1; i <= ndet; i++) {
            if ((int)centroids_x[i] == -1) { continue; }
            if (det2cat[i] > 30) { continue; }
            apc::Recognized rec;
            rec.is3d = mask_msg.is3d;
            rec.job_number = job_number;
            rec.obj_id = det2cat[i];
            rec.confidence = det2score[i];
            rec.angle_ma = angles_ma[i];
            rec.angle_mi = angles_mi[i];
            rec.centroid_x = centroids_x[i];
            rec.centroid_y = centroids_y[i];
            objects.push_back(rec);
            ROS_INFO("> found %d at (%.02f, %.02f)", det2cat[i],
                     centroids_x[i], centroids_y[i]);
        }
        apc::Recognition rec_msg;
        rec_msg.recognitions = objects;
        rec_pub.publish(rec_msg);
    }
}

// Receives the actual mask image from the ROS/Matlab link
void AnalyzeMask::receive_mask(const sensor_msgs::ImageConstPtr& mask) {
    ROS_INFO("> received mask");
    cv_bridge::CvImagePtr cv_ptr;
    ROS_INFO("> initiated cv_bridge");
    try {
        cv_ptr = cv_bridge::toCvCopy(mask, "mono8");
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("! exception while receiving mask");
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    ROS_INFO("> bridge conversion good");
    if (cv_ptr->image.rows == 400) {
        ROS_INFO("> copying 2d mask");
        uchar* s = cv_ptr->image.data;
        uchar* d = mask_2d.data;
        for (int i = 0; i < 400*640; i++) {
            d[i] = s[i];
        }
    } else if (cv_ptr->image.rows == 480) {
        ROS_INFO("> copying 3d mask");
        uchar* s = cv_ptr->image.data;
        uchar* d = mask_3d.data;
        for (int i = 0; i < 480*640; i++) {
            d[i] = s[i];
        }
    } else {
        ROS_ERROR("! error while receiving mask");
        ROS_ERROR("! image incorrect size");
    }
    ROS_INFO("> converted mask");
    if (cv_ptr->image.rows == 480) {
        mask_3d = remap(mask_3d);
    }
    ROS_INFO("> remapped mask");
}

void AnalyzeMask::receive_scene(const sensor_msgs::PointCloud2& cloud) {
    Cloud tmp (new pcl::PointCloud<pcl::PointXYZ>);
    /*
    // convert and transform the point cloud
    sensor_msgs::PointCloud  p1;
    sensor_msgs::PointCloud  p2;
    sensor_msgs::PointCloud2 p3;
    sensor_msgs::convertPointCloud2ToPointCloud(cloud, p1);
    ROS_INFO("> size %d", (int)p1.points.size());
    transform.waitForTransform("torso", cloud.header.frame_id, 
                               cloud.header.stamp, ros::Duration(10));
    transform.transformPointCloud("torso", p1, p2);
    ROS_INFO("> size %d", (int)p2.points.size());
    sensor_msgs::convertPointCloudToPointCloud2(p2, p3);
    pcl::fromROSMsg(p3, *tmp);
    */
    
    pcl::fromROSMsg(cloud, *tmp);
    //pcl::transformPointCloud(*tmp, *tmp, transform);
    
    scene = tmp;
    scene_header = cloud.header;
}

// Return a vector of point clouds, where each point cloud represents a
// detection
std::vector<Cloud> AnalyzeMask::apply_mask() {
    ROS_INFO("> applying mask to point cloud");
    if (!scene) {
        ROS_INFO("! no point cloud");
    }
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> objects;

    for (int i = 0; i <= ndet; i++) {
        Cloud obj (new pcl::PointCloud<pcl::PointXYZ>);
        obj->width = scene->width;
        obj->height = scene->height;
        obj->resize(obj->width * obj->height);
        obj->is_dense = scene->is_dense;
        objects.push_back(obj);
    }
    ROS_INFO("> created cluster storage");

    float nan = std::numeric_limits<float>::quiet_NaN();
    pcl::PointXYZ nan_p = pcl::PointXYZ(nan, nan, nan);

    ROS_INFO("> processing cloud...");
    ROS_INFO("> height %d, width %d", scene->height, scene->width);

    int ct = 0;
    for (int x = 0; x < mask_3d.rows; x++) {
        if (x > 0 && (x % 48) == 0) {
            ROS_INFO("> %d%% done...", (x/48*10));
        }
        for (int y = 0; y < mask_3d.cols; y++, ct++) {
            //pcl::PointXYZ p = scene->points[y*x+x];
            pcl::PointXYZ p = scene->points[ct];

            // get corresponding mask value (detection id)
            int det = (int)mask_3d.at<uchar>(x, y);

            for (int i = 1; i <= ndet; i++) {
                if (i == det) {
                    objects[i]->points[ct].x = p.x;
                    objects[i]->points[ct].y = p.y;
                    objects[i]->points[ct].z = p.z;
                } else {
                    objects[i]->points[ct].x = nan;
                    objects[i]->points[ct].y = nan;
                    objects[i]->points[ct].z = nan;
                }
            }
        }
    }

    ROS_INFO("> applied mask");
    return objects;
}

// Returns centroid of each detected cluster
// Return vector is same length as # of detections
// Each element is a vector for a detection
// Order is centroid x, centroid y, centroid z
std::vector<std::vector<float> > AnalyzeMask::analyze_3d(std::vector<Cloud> objects) {
    ROS_INFO("> doing 3d analysis");
    std::vector<std::vector<float> > statistics;

    for (int i = 0; i < objects.size(); i++) {
        std::vector<float> stat;
        if (i == 0 || (int)centroids_x[i] == -1) {
            statistics.push_back(stat);
            continue;
        }
        Cloud obj = objects[i];
        int ct = 0;
        for (int j = 0; j < obj->points.size(); j++) {
            if (!isnan(obj->points[j].x)) { ct++; }
        }
        ROS_INFO("> cloud %d contains %d points", i, ct);

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*obj, *obj, indices);

	/*
        ROS_INFO("> removing cluster outliers");
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> filt;
	Cloud tmp (new pcl::PointCloud<pcl::PointXYZ>);
        filt.setInputCloud(obj);
        filt.setRadiusSearch(0.5);
        filt.setMinNeighborsInRadius(2);
        filt.filter(*tmp);
	*/

        ROS_INFO("> finding cluster centroid");
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*obj, centroid);

        // add centroid values
        stat.push_back(centroid[0]);
        stat.push_back(centroid[1]);
        stat.push_back(centroid[2]);

        statistics.push_back(stat);
    }

    ROS_INFO("> completed 3d analysis");
    return statistics;
}

cv::Mat AnalyzeMask::remap(cv::Mat mask) {
    ROS_INFO("> starting remap");
    cv::Mat Kd = (cv::Mat_<float>(3, 3) << 575.8157, 0, 314.5, 0, 575.8157, 235.5, 0, 0, 1);
    cv::Mat Kdi = Kd.inv();
    cv::Mat Kc = (cv::Mat_<float>(3, 3) << 525, 0, 319.5, 0, 525, 239.5, 0, 0, 1);

    cv::Mat rect = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC1);

    cv::Mat p = cv::Mat(3, 1, CV_32FC1, (float)1);
    cv::Mat q = cv::Mat(3, 1, CV_32FC1, (float)1);
    int x_, y_;

    for (int x = 0; x < mask.cols; x++) {
        if (x > 0 && (x % 64) == 0) {
            ROS_INFO("> %d%% done...", (x/64*10));
        }
        for (int y = 0; y < mask.rows; y++) {
            p.at<float>(0, 0) = x;
            p.at<float>(1, 0) = y;

            q = Kdi * Kc * p;
            x_ = round(q.at<float>(0, 0));
            y_ = round(q.at<float>(1, 0));

            if (x_ >= 0 && x_ < mask.cols && y_ >= 0 && y_ < mask.rows) {
                rect.at<uchar>(y, x) = mask.at<uchar>(y_, x_);
            }
        }
    }

    return rect;
}


int main(int argc, char** argv) {
    ROS_INFO("> init mask analysis node");

    ros::init(argc, argv, "AnalyzeMask");
    AnalyzeMask analyzer;
    ros::spin();
}

