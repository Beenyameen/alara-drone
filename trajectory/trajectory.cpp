#include <iostream>
#include <zmq.hpp>
#include <opencv2/opencv.hpp>
#include <System.h>

using namespace std;

int main() {
    ORB_SLAM3::System slam("/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt", "settings.yaml", ORB_SLAM3::System::RGBD, false);

    zmq::context_t ctx(1);
    zmq::socket_t sub(ctx, ZMQ_SUB);
    sub.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    int hwm = 2;
    sub.setsockopt(ZMQ_RCVHWM, &hwm, sizeof(hwm));
    sub.connect("tcp://broker:12000");

    zmq::socket_t pub(ctx, ZMQ_PUB);
    pub.setsockopt(ZMQ_SNDHWM, &hwm, sizeof(hwm));
    pub.bind("tcp://0.0.0.0:13000");
    
    while (true) {
        zmq::message_t m0, m1, m2;
        sub.recv(&m0);
        sub.recv(&m1);
        sub.recv(&m2);

        double ts = *static_cast<double*>(m0.data());
        cv::Mat color = cv::imdecode(std::vector<uchar>(static_cast<uchar*>(m1.data()), static_cast<uchar*>(m1.data()) + m1.size()), cv::IMREAD_COLOR);
        cv::Mat depth(480, 640, CV_16U, m2.data());

        Sophus::SE3f pose = slam.TrackRGBD(color, depth, ts);
        int state = slam.GetTrackingState();

        if (state == 2) {
            Eigen::Matrix4f mat = pose.matrix();

            zmq::message_t out(64);
            memcpy(out.data(), mat.data(), 64);
            pub.send(out);
        }
    }
}
