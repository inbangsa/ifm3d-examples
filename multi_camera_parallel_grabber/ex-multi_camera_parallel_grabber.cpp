/*
 * Copyright (C) 2019 ifm electronic, gmbh
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

//
// ex-multi_camera_parallel_grabber.cpp
//
// setup : refer README.md file
//
// Prerequisites:
// *) one ifm 3D camera should be configured to use "Process Interface" for trigger.
// *) All other camera must be in hardware trigger mode
// *)
//

#include <iostream>
#include <memory>
#include <chrono>
#include <vector>
#include <thread>
#include <ifm3d/camera.h>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>

namespace
{   // enum for all the trigger types supported by o3d3xx device
    enum class trigger_mode : int { FREE_RUN = 1, SW = 2, POSITIVE_EDGE = 3, NEGATIVE_EDGE = 4, POSITIVE_N_NEGATIVE = 5 };

    //CHANGE IP addreses to those of camera's avaialible!!!!
    const auto CAMERA0 = "192.168.0.70"; // Device in software trigger mode
    const auto CAMERA1 = "192.168.0.71";
    const auto CAMERA2 = "192.168.0.72";

    //Add the IP of cameras to be used with hardware trigger
    const std::vector<std::string> camera_ips = {CAMERA1,CAMERA2};
    int num_cam_finished = camera_ips.size();

    auto j_conf = json::parse(R"(
         {
           "ifm3d":
           {
             "Device":
              {
                "ActiveApplication": "1"
              },
              "Apps":
              [
                {
                  "Index": "1",
                  "TriggerMode":"2",
                  "LogicGraph": "{\"IOMap\": {\"OUT1\": \"RFT\",\"OUT2\": \"AQUFIN\"},\"blocks\": {\"B00001\": {\"pos\": {\"x\": 200,\"y\": 200},\"properties\": {},\"type\": \"PIN_EVENT_IMAGE_ACQUISITION_FINISHED\"},\"B00002\": {\"pos\": {\"x\": 200,\"y\": 75},\"properties\": {},\"type\": \"PIN_EVENT_READY_FOR_TRIGGER\"},\"B00003\": {\"pos\": {\"x\": 600,\"y\": 75},\"properties\": {\"pulse_duration\": 0},\"type\": \"DIGITAL_OUT1\"},\"B00005\": {\"pos\": {\"x\": 600,\"y\": 200},\"properties\": {\"pulse_duration\": 0},\"type\": \"DIGITAL_OUT2\"}},\"connectors\": {\"C00000\": {\"dst\": \"B00003\",\"dstEP\": 0,\"src\": \"B00002\",\"srcEP\": 0},\"C00001\": {\"dst\": \"B00005\",\"dstEP\": 0,\"src\": \"B00001\",\"srcEP\": 0}}}",
                  "Imager":
                  {
                      "ExposureTime": "1000",
                      "Type":"under5m_moderate",
                      "FrameRate":"20"
                  }
                }
             ]
            }
          }
      )");

    //for configuration of required trigger on camera
    void configuration(ifm3d::Camera::Ptr& camera, trigger_mode type)
    {
        try
        {
            auto application_id = camera->ActiveApplication();
            j_conf["ifm3d"]["Device"]["ActiveApplication"] = std::to_string(application_id);
            j_conf["ifm3d"]["Apps"][0]["Index"] = std::to_string(application_id);
            j_conf["ifm3d"]["Apps"][0]["TriggerMode"] = std::to_string((int)type);

            camera->FromJSON(j_conf);
        }
        catch (std::exception &e)
        {
            std::cout << e.what();
        }
    }

    // for grabbing data when its available
    void grabimage(ifm3d::FrameGrabber::Ptr& frame_grabber,
            std::function<void(ifm3d::ImageBuffer::Ptr image_buffer,
                bool in_time)> callback)
    {
        while(true)
        {
            std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(1));
            ifm3d::ImageBuffer::Ptr image_buffer = std::make_shared<ifm3d::ImageBuffer>();
            const bool in_time = frame_grabber->WaitForFrame(image_buffer.get(), 10000);
            callback(image_buffer,in_time);
        }
    };
}; //end namespcase

int main(int argc, const char **argv)
{
    //vectors for the objects to be used.
    std::vector<std::thread> workers;

    // Connecting to device with software trigger
    auto cam_sw = ifm3d::Camera::MakeShared(CAMERA0);
    auto framegrab_sw = std::make_shared<ifm3d::FrameGrabber>(cam_sw, ifm3d::IMG_AMP );

    // configure the CAMERA0 as software trigger
    configuration(cam_sw, trigger_mode::SW);

    auto buf_callback =
        [&](auto image_buffer, bool in_time) -> void
    {
        if(in_time)
        {
            std::cout << "Frame received: "
                << image_buffer->TimeStamp().time_since_epoch().count()
                << std::endl;
        }
        else
        {
            std::cout << "Timeout occured" << std::endl;
        }
    };
    auto trig_callback =
        [&](auto image_buffer, bool in_time) -> void
    {
        // Trigger the next frame
        framegrab_sw->SWTrigger();
        // execute the regular callback
        buf_callback(image_buffer,in_time);

    };

    // queue the software triggered camera
    workers.push_back(
            std::thread(std::bind(grabimage,framegrab_sw,buf_callback))
            );

    // Create ifm3d objects of Camera, ImageBuffer and FrameGrabber for each of the camera devices in hardware trigger.
    for(auto& camera_ip:camera_ips)
    {
        auto cam = ifm3d::Camera::MakeShared(camera_ip);
        configuration(cam, trigger_mode::POSITIVE_EDGE);
        auto frame_grabber = std::make_shared<ifm3d::FrameGrabber>(cam, ifm3d::IMG_AMP );
        if(camera_ip!=*(camera_ips.end() -1))
        {
            workers.push_back(
                    std::thread(
                        std::bind(grabimage, frame_grabber,buf_callback))
                    );
        }
        else
        {
            workers.push_back(
                    std::thread(
                        std::bind(grabimage, frame_grabber,trig_callback))
                    );
        }
    }

    // Kickstart the loop
    framegrab_sw->SWTrigger();

    //waiting for all threads to complete
    for (auto &worker: workers)
    {
        if (worker.joinable())
            worker.join();
    }
    return 0;
}
