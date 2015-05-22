#include <ros/ros.h>

// point cloud stuff
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

#include <pcl/point_types.h>
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
#include <apc/RecInfo.h>
#include <apc/Recognized.h>
#include <apc/Recognition.h>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud;

class AnalyzeMask {
    public: 
        AnalyzeMask();
        void receive_mask_info(apc::RecInfo);
        void receive_scene(const sensor_msgs::PointCloud2&);
        void receive_mask(const sensor_msgs::ImageConstPtr&);
    private:
        cv::Mat mask_3d;
        cv::Mat mask_2d;
        Cloud scene;
        int ndet;
        std::vector<int> det2cat;
        std::vector<float> det2score;

        ros::Subscriber mask_info_sub;
        ros::Subscriber scene_sub;
        image_transport::Subscriber mask_sub;
        ros::Publisher rec_pub;

        ros::NodeHandle nh;
        image_transport::ImageTransport itnh;
        tf::TransformListener transform;

        std::vector<Cloud> apply_mask();
        std::vector<std::vector<float> > analyze_2d(bool);
        std::vector<std::vector<float> > analyze_3d(std::vector<Cloud>);
};

AnalyzeMask::AnalyzeMask() : itnh(nh) {
    mask_3d = cv::Mat::zeros(640, 480, CV_8UC1);
    mask_2d = cv::Mat::zeros(640, 400, CV_8UC1);
 
    mask_info_sub = nh.subscribe("/apc/recognition_information", 1, 
                                 &AnalyzeMask::receive_mask_info, this);
    scene_sub = nh.subscribe("/apc/recognition_pcl", 1, 
                             &AnalyzeMask::receive_scene, this);
    mask_sub = itnh.subscribe("/apc/recognition_mask_image", 1, 
                              &AnalyzeMask::receive_mask, this);
    rec_pub = nh.advertise<apc::Recognition>("/apc/recognition", 1);
}

// Receive flag for 2d/3d, point cloud if necessary, and two vectors
void AnalyzeMask::receive_mask_info(apc::RecInfo mask_msg) {
    det2cat = mask_msg.categories;
    det2score = mask_msg.scores;
    ndet = mask_msg.number_of_items;

    if (mask_msg.is3d) {
        // pull out info
        std::vector<Cloud> clusters = apply_mask();
        std::vector<std::vector<float> > data_2d = analyze_2d(true);
        std::vector<std::vector<float> > data_3d = analyze_3d(clusters);

        // publish
        std::vector<apc::Recognized> objects;
        for (int i = 0; i < ndet; i++) {
            if (data_2d[i].size() == 0) { continue; }
            apc::Recognized rec;
            rec.obj_id = det2cat[i];
            rec.confidence = det2score[i];
            rec.centroid_x = data_3d[i][0];
            rec.centroid_y = data_3d[i][1];
            rec.centroid_z = data_3d[i][2];
            rec.angle_ma = data_2d[i][0];
            rec.angle_mi = data_2d[i][1];
            objects.push_back(rec);
        }
        apc::Recognition rec_msg;
        rec_msg.recognitions = objects;
        rec_pub.publish(rec_msg);
    } else {
        // only get 2d info
        std::vector<std::vector<float> > data_2d = analyze_2d(false);
        std::vector<apc::Recognized> objects;
        for (int i = 0; i < ndet; i++) {
            if (data_2d[i].size() == 0) { continue; }
            apc::Recognized rec;
            rec.obj_id = det2cat[i];
            rec.confidence = det2score[i];
            rec.angle_ma = data_2d[i][0];
            rec.angle_mi = data_2d[i][1];
            rec.centroid_x = data_2d[i][2];
            rec.centroid_y = data_2d[i][3];
            objects.push_back(rec);
        }
        apc::Recognition rec_msg;
        rec_msg.recognitions = objects;
        rec_pub.publish(rec_msg);
    }
}

// Receives the actual mask image from the ROS/Matlab link
void AnalyzeMask::receive_mask(const sensor_msgs::ImageConstPtr& mask) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(mask, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("! exception while receiving mask");
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    if (cv_ptr->image.rows == 400) {
        mask_2d = cv_ptr->image;
    } else if (cv_ptr->image.rows == 480) {
        mask_3d = cv_ptr->image;
    } else {
        ROS_ERROR("! error while receiving mask");
        ROS_ERROR("! image incorrect size");
    }
}

void AnalyzeMask::receive_scene(const sensor_msgs::PointCloud2& cloud) {
    // convert and transform the point cloud
    sensor_msgs::PointCloud  p1;
    sensor_msgs::PointCloud  p2;
    sensor_msgs::PointCloud2 p3;
    sensor_msgs::convertPointCloud2ToPointCloud(cloud, p1);
    transform.transformPointCloud("torso", p1, p2);
    sensor_msgs::convertPointCloudToPointCloud2(p1, p3);
    pcl::fromROSMsg(p3, *scene);
}

// Return a vector of point clouds, where each point cloud represents a
// detection
std::vector<Cloud> AnalyzeMask::apply_mask() {
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> objects;

    for (int i = 0; i < ndet; i++) {
        Cloud obj (new pcl::PointCloud<pcl::PointXYZ>);
        obj->width = scene->width;
        obj->height = scene->height;
        obj->resize(obj->width * obj->height);
        obj->is_dense = false;
        objects.push_back(obj);
    }

    float nan = std::numeric_limits<float>::quiet_NaN();
    pcl::PointXYZ nan_p = pcl::PointXYZ(nan, nan, nan);

    for (int x = 0; x < scene->width; x++) {
        for (int y = 0; y < scene->height; y++) {
            pcl::PointXYZ p = scene->points[y*x+x];

            // get corresponding mask value (detection id)
            int det = mask_3d.at<int>(x, y);

            for (int i = 0; i < ndet; i++) {
                if (i == det) {
                    objects[i]->points[y*x+x] = p;
                } else {
                    objects[i]->points[y*x+x] = nan_p;
                }
            }
        }
    }

    return objects;
}

// For each detection, return the angles of the major and minor axis
// computed using a rotated bounding box, and a centroid
// All angles are upright (from 270 to 0 to 90)
// Return vector is always as long as # of detections
// Each element is a vector for a detection
// If a detection vector is empty, OpenCV couldn't find the region in the mask
// Otherwise, order is major angle, minor angle, centroid x, centroid y
std::vector<std::vector<float> > AnalyzeMask::analyze_2d(bool is3d) {
    cv::Mat mask;
    if (is3d) {
        mask = mask_3d;
    } else {
        mask = mask_2d;
    }
    std::vector<std::vector<float> > analysis;

    for (int i = 0; i < ndet; i++) {
        std::vector<float> data;
        cv::Mat reg = mask == i;

        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(reg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        int max_size = 0;
        int max_idx = -1;
        for (int j = 0; j < contours.size(); j++) {
            if (contours[j].size() > max_size) {
                max_size = contours[j].size();
                max_idx = j;
            }
        }
        if (max_idx == -1) {
            analysis.push_back(data);
            continue;
        }

        cv::RotatedRect bb = cv::minAreaRect(contours[max_idx]);
        float a = bb.angle;
        float b = a - 90;
        // map angle to be upright (between 270 to 0 to 90 degrees)
        if (a > 90 && a <= 180) { a = a + 180; }
        if (a > 180 && a < 270) { a = a - 180; }
        if (b > 90 && b <= 180) { b = b + 180; }
        if (b > 180 && b < 270) { b = b - 180; }

        if (bb.size.height >= bb.size.width) {
            data.push_back(a);
            data.push_back(b);
        } else {
            data.push_back(b);
            data.push_back(a);
        }

        cv::Moments moments = cv::moments(contours[max_idx]);
        data.push_back((int)(moments.m10 / moments.m00));
        data.push_back((int)(moments.m01 / moments.m00));

        analysis.push_back(data);
    }

    return analysis;
}

// Returns centroid of each detected cluster
// Return vector is same length as # of detections
// Each element is a vector for a detection
// Order is centroid x, centroid y, centroid z
std::vector<std::vector<float> > AnalyzeMask::analyze_3d(std::vector<Cloud> objects) {
    std::vector<std::vector<float> > statistics;

    for (int i = 0; i < objects.size(); i++) {
        Cloud obj = objects[i];
        std::vector<float> stat;

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*obj, *obj, indices);

        pcl::RadiusOutlierRemoval<pcl::PointXYZ> filt;
        filt.setInputCloud(obj);
        filt.setRadiusSearch(0.5);
        filt.setMinNeighborsInRadius(2);
        filt.filter(*obj);

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*obj, centroid);

        /*
        // naive way to compute width/height
        pcl::PointXYZ min;
        pcl::PointXYZ max;
        pcl::getMinMax3D(*obj, *min, *max);
        float width = max.y - min.y;
        float height = max.z - min.z;
        */

        // add centroid values
        stat.push_back(centroid[0]);
        stat.push_back(centroid[1]);
        stat.push_back(centroid[2]);

        statistics.push_back(stat);
    }
    return statistics;
}

int main(int argc, char** argv) {
    ROS_INFO(" init mask analysis node");

    ros::init(argc, argv, "AnalyzeMask");

    AnalyzeMask analyzer;

    ros::spin();
}

