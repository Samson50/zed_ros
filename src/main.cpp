///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2016, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////


/************************************************************************************
 ** This sample demonstrates how to use PCL (Point Cloud Library) with the ZED SDK **
 ************************************************************************************/

#include <stdio.h>
#include <string.h>
#include <ctime>
#include <chrono>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

#ifdef _WIN32
#undef max
#undef min
#endif

#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common_headers.h>

using namespace sl::zed;
using namespace std;

typedef struct image_bufferStruct {
    float* data_cloud;
    std::mutex mutex_input;
    int width, height;
} image_buffer;


Camera* zed;
image_buffer* buffer;
SENSING_MODE dm_type = STANDARD;
bool stop_signal;

// Grab called in a thread to parallelize the rendering and the computation

void grab_run() {
    float* p_cloud;

    while (!stop_signal) {
        if (!zed->grab(dm_type)) {
            p_cloud = (float*) zed->retrieveMeasure_gpu(MEASURE::XYZRGBA).data; // Get the pointer
            // Fill the buffer
            buffer->mutex_input.lock(); // To prevent from data corruption
            memcpy(buffer->data_cloud, p_cloud, buffer->width * buffer->height * sizeof (float) * 4);
            buffer->mutex_input.unlock();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}


int main(int argc, char** argv) {
    stop_signal = false;

    if (argc > 2) {
        std::cout << "Only the path of a SVO can be passed in arg" << std::endl;
        return -1;
    }

    if (argc == 1) // Live Mode
        zed = new Camera(VGA);
    else // SVO playback mode
        zed = new Camera(argv[1]);

    sl::zed::InitParams params;
    params.mode = PERFORMANCE;
    params.unit = METER; // Scale to fit OpenGL world
    params.coordinate = RIGHT_HANDED; // OpenGL compatible
    params.verbose = true;

    ERRCODE err = zed->init(params);
    cout << errcode2str(err) << endl;
    if (err != SUCCESS) {
        delete zed;
        return 1;
    }
	
	// ROS Things
	ros::NodeHandle nh = getMTNodeHandle();
	sensor_msgs::PointCloud2 output;
	string point_cloud_topic = "point_cloud/cloud_registered";
	std::string point_cloud_frame_id = "/zed_current_frame";
    ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2> (point_cloud_topic, 1);
	ros::Rate loop_rate(30);

    int width = zed->getImageSize().width;
    int height = zed->getImageSize().height;
	int size = width*height;

    // Allocate data
    buffer = new image_buffer();
    buffer->height = height;
    buffer->width = width;
    buffer->data_cloud = new float[buffer->height * buffer->width * 4];

    int size = height*width;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    float* data_cloud;
    // Run thread
    std::thread grab_thread(grab_run);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    float color;
    int index4 = 0;

    point_cloud_ptr->points.resize(size);

    while (nh.ok()) {

        if (buffer->mutex_input.try_lock()) {
            data_cloud = buffer->data_cloud;
            index4 = 0;

            for (int i = 0; i < size; i++) {
                if (cpu_cloud[index4 + 2] > 0) { // Check if it's an unvalid point, the depth is more than 0
                    index4 += 4;
                    continue;
                }
                point_cloud.points[i].y = -cpu_cloud[index4++];
                point_cloud.points[i].z = cpu_cloud[index4++];
                point_cloud.points[i].x = -cpu_cloud[index4++];
                point_cloud.points[i].rgb = cpu_cloud[index4++];
            }
            buffer->mutex_input.unlock();
            pcl::toROSMsg(point_cloud, output); // Convert the point cloud to a ROS message
            output.header.frame_id = point_cloud_frame_id; // Set the header values of the ROS message
            output.header.stamp = ros::Time::now();;
            output.height = height;
            output.width = width;
            output.is_bigendian = false;
            output.is_dense = false;
            pub_cloud.publish(output);
        }
		loop_rate.sleep();
        //std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Stop the grabbing thread
    stop_signal = true;
    grab_thread.join();

    delete[] buffer->data_cloud;
    delete buffer;
    delete zed;
    return 0;
}
