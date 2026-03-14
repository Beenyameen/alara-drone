#include <iostream>
#include <vector>
#include <mutex>
#include <thread>
#include <queue>
#include <zmq.hpp>
#include <opencv2/opencv.hpp>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct FrameData {
    double ts;
    cv::Mat color;
    cv::Mat depth;
    rtabmap::Transform pose;
};

std::mutex cloud_mutex;
std::vector<float> point_cloud;

void receive_loop(std::queue<FrameData>& buffer, std::mutex& buffer_mutex) {
    zmq::context_t ctx(1);
    zmq::socket_t sub(ctx, ZMQ_SUB);
    sub.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    int hwm = 0; // Infinite buffer for zero message drop
    sub.setsockopt(ZMQ_RCVHWM, &hwm, sizeof(hwm));
    sub.connect("tcp://trajectory:14000");

    while (true) {
        zmq::message_t m0, m1, m2, m3;
        sub.recv(&m0);
        sub.recv(&m1);
        sub.recv(&m2);
        sub.recv(&m3);

        FrameData fd;
        fd.ts = *static_cast<double*>(m0.data());
        fd.color = cv::imdecode(std::vector<uchar>(static_cast<uchar*>(m1.data()), static_cast<uchar*>(m1.data()) + m1.size()), cv::IMREAD_COLOR);
        fd.depth = cv::Mat(480, 640, CV_16U, m2.data()).clone();
        
        float* p = static_cast<float*>(m3.data());
        fd.pose = rtabmap::Transform(
            p[0], p[4], p[8],  p[12],
            p[1], p[5], p[9],  p[13],
            p[2], p[6], p[10], p[14]
        ).inverse();

        std::lock_guard<std::mutex> lock(buffer_mutex);
        buffer.push(fd);

    }
}

void rep_loop() {
    zmq::context_t ctx(1);
    zmq::socket_t rep(ctx, ZMQ_REP);
    rep.bind("tcp://0.0.0.0:15000");

    while (true) {
        zmq::message_t req;
        rep.recv(&req);
        
        std::lock_guard<std::mutex> lock(cloud_mutex);

        zmq::message_t rep_msg(point_cloud.size() * sizeof(float));
        memcpy(rep_msg.data(), point_cloud.data(), point_cloud.size() * sizeof(float));
        rep.send(rep_msg);
    }
}

int main() {
    std::queue<FrameData> buffer;
    std::mutex buffer_mutex;

    std::thread t1(receive_loop, std::ref(buffer), std::ref(buffer_mutex));
    std::thread t2(rep_loop);

    rtabmap::Rtabmap rtabmap;
    rtabmap::ParametersMap parameters;
    parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapTimeThr(), "0"));
    parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapDetectionRate(), "0"));
    parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemRehearsalSimilarity(), "1.0"));
    parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDEnabled(), "true"));

    rtabmap.init(parameters, "");

    rtabmap::CameraModel model(500.0, 500.0, 320.0, 240.0, rtabmap::Transform::getIdentity(), 0, cv::Size(640, 480));

    int id = 1;
    std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> local_clouds;

    while (true) {
        FrameData fd;
        bool has_data = false;
        {
            std::lock_guard<std::mutex> lock(buffer_mutex);
            if (!buffer.empty()) {
                fd = buffer.front();
                buffer.pop();
                has_data = true;
            }
        }

        if (has_data) {
            rtabmap::SensorData data(fd.color, fd.depth, model, id, fd.ts);
            
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_cloud = rtabmap::util3d::cloudRGBFromSensorData(data, 4, 4.0f, 0.0f);
            
            // Push un-decimated cloud to global map cache
            local_clouds[id] = local_cloud;

            rtabmap.process(data, fd.pose);
            id++;

            std::map<int, rtabmap::Transform> poses;
            std::multimap<int, rtabmap::Link> links;
            rtabmap.getGraph(poses, links, true, true);

            std::vector<float> new_cloud;
            for (auto& pair : poses) {
                int nId = pair.first;
                rtabmap::Transform pose = pair.second;
                
                if (local_clouds.count(nId)) {
                    auto cloud = rtabmap::util3d::transformPointCloud(local_clouds[nId], pose);
                    for (const auto& pt : cloud->points) {
                        if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
                            new_cloud.push_back(pt.x);
                            new_cloud.push_back(pt.y);
                            new_cloud.push_back(pt.z);
                            new_cloud.push_back(pt.r / 255.0f);
                            new_cloud.push_back(pt.g / 255.0f);
                            new_cloud.push_back(pt.b / 255.0f);
                        }
                    }
                }
            }
            
            {
                std::lock_guard<std::mutex> lock(cloud_mutex);
                point_cloud = std::move(new_cloud);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}
