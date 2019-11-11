// Posefusion.cpp
// Uses the OpenPose API to obtain pose data from webcam and publishes the data
// over MQTT to generate 3D pose estimation

#include <openpose/flags.hpp>
#include <openpose/headers.hpp>
#include "mqtt/client.h"
#include <sys/time.h>

// Custom OpenPose flags
DEFINE_bool(no_display, false, "Disable the visual display.");

const std::string SERVER_ADDR = "oz.andrew.cmu.edu";
const std::string CLIENT_ID   = "lambda-3";
const std::string TOPIC       = "/lambda/3/pose";
const float MIN_CONF_SCORE    = 0.4;
const float MAX_TIME_FRAME_MS = 40;

const int QOS = 0;

const mqtt::message EXIT_MSG = mqtt::message(TOPIC, "EXIT", 4, 2, false);

mqtt::client client(SERVER_ADDR, CLIENT_ID);

static long int last_sent = 0;

// This worker gets all the points and publishes them to a MQTT broker
class WUserOutput : public op::WorkerConsumer<std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>>
{
public:

    void initializationOnThread()
    {
        try
        {
            std::cout << "Initialzing MQTT Client..." << std::flush;

            mqtt::connect_options connOpts;
            connOpts.set_keep_alive_interval(20);
            connOpts.set_clean_session(true);
            std::cout << "OK" << std::endl;

            std::cout << "Connecting to MQTT Server..." << std::flush;
            client.connect(connOpts);
            std::cout << "OK" << std::endl;
        }
        catch (const mqtt::exception& exc)
        {
            std::cerr << exc.what() << std::endl;
            exit(0);
        }
    }

    void workConsumer(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
    {
        try
        {
            // User's displaying/saving/other processing here
            // datumPtr->cvOutputData: rendered frame with pose or heatmaps
            // datumPtr->poseKeypoints: Array<float> with the estimated pose
            if (datumsPtr != nullptr && !datumsPtr->empty())
            {
                const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
                const auto& poseScores = datumsPtr->at(0)->poseScores;
                // client.publish(mqtt::message(TOPIC, "Body keypoints:", QOS, false));
                std::string total_message = "";
                for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
                {
		    float score = poseScores[person];
		    printf("Score %d: %f\n", person, poseScores[person]);
		    if (score > MIN_CONF_SCORE) {
			    struct timeval tp;
			    gettimeofday(&tp, NULL);
			    long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
			    last_sent = ms;

			    // Begin to build message to send over MQTT
			    std::string message = std::to_string(ms) + " " + std::to_string(person) + " " + std::to_string(score) + " ";

			    for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
			    {
				    // Only iterate through first 2 to get xy coordinates, 3 to include score
				    for (auto xyscore = 0 ; xyscore < 2; xyscore++)
				    {
					    message += std::to_string(poseKeypoints[{person, bodyPart, xyscore}]) + " ";
				    }
			    }
			    total_message += message + ",";
		    }
                }
		// Publish once message is fully built
		client.publish(mqtt::message(TOPIC, total_message, QOS, false));

                // We assume that we are running OpenPose WITHOUT face and hand detection
                if (FLAGS_face) {
                    op::log("Face keypoints: " + datumsPtr->at(0)->faceKeypoints.toString());
                }
                if (FLAGS_hand){
                    op::log("Left hand keypoints: " + datumsPtr->at(0)->handKeypoints[0].toString());
                    op::log("Right hand keypoints: " + datumsPtr->at(0)->handKeypoints[1].toString());
                }

                // End program if ESC key is pressed
                const char key = (char)cv::waitKey(1);
                if (key == 27) {
                    std::cout << "Disconnecting MQTT..." << std::flush;
                    client.publish(EXIT_MSG);
                    client.disconnect();
                    std::cout << "OK" << std::endl;
                    this->stop();
                }
            } else {
		std::string total_message = "";
		struct timeval tp;
		gettimeofday(&tp, NULL);
		long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
		if (ms - last_sent > MAX_TIME_FRAME_MS) {
			// Begin to build message to send over MQTT
			std::string message = std::to_string(ms);
			total_message += message;
			client.publish(mqtt::message(TOPIC, total_message, QOS, false));
			last_sent = ms;
		}
	    }
        }
        catch (const std::exception& e)
        {
            this->stop();
            op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }
};

void configureWrapper(op::Wrapper& opWrapper)
{
    try
    {
        // Configuring OpenPose

        // logging_level
        op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.",
                  __LINE__, __FUNCTION__, __FILE__);
        op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
        op::Profiler::setDefaultX(FLAGS_profile_speed);

        // Applying user defined configuration - GFlags to program variables
        // producerType
        op::ProducerType producerType;
        std::string producerString;
        std::tie(producerType, producerString) = op::flagsToProducer(
            FLAGS_image_dir, FLAGS_video, FLAGS_ip_camera, FLAGS_camera, FLAGS_flir_camera, FLAGS_flir_camera_index);
        // cameraSize
        const auto cameraSize = op::flagsToPoint(FLAGS_camera_resolution, "-1x-1");
        // outputSize
        const auto outputSize = op::flagsToPoint(FLAGS_output_resolution, "-1x-1");
        // netInputSize
        const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "-1x368");
        // faceNetInputSize
        const auto faceNetInputSize = op::flagsToPoint(FLAGS_face_net_resolution, "368x368 (multiples of 16)");
        // handNetInputSize
        const auto handNetInputSize = op::flagsToPoint(FLAGS_hand_net_resolution, "368x368 (multiples of 16)");
        // poseMode
        const auto poseMode = op::flagsToPoseMode(FLAGS_body);
        // poseModel
        const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);
        // JSON saving
        if (!FLAGS_write_keypoint.empty())
            op::log("Flag `write_keypoint` is deprecated and will eventually be removed."
                    " Please, use `write_json` instead.", op::Priority::Max);
        // keypointScaleMode
        const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);
        // heatmaps to add
        const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
                                                      FLAGS_heatmaps_add_PAFs);
        const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
        // >1 camera view?
        const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1 || FLAGS_flir_camera);
        // Face and hand detectors
        const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);
        const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);
        // Enabling Google Logging
        const bool enableGoogleLogging = true;

        // Initializing the user custom classes
        auto wUserOutput = std::make_shared<WUserOutput>();
        // Add custom processing
        const auto workerOutputOnNewThread = true;
        opWrapper.setWorker(op::WorkerType::Output, wUserOutput, workerOutputOnNewThread);

        // Pose configuration (use WrapperStructPose{} for default and recommended configuration)
        const op::WrapperStructPose wrapperStructPose{
            poseMode, netInputSize, outputSize, keypointScaleMode, FLAGS_num_gpu, FLAGS_num_gpu_start,
            FLAGS_scale_number, (float)FLAGS_scale_gap, op::flagsToRenderMode(FLAGS_render_pose, multipleView),
            poseModel, !FLAGS_disable_blending, (float)FLAGS_alpha_pose, (float)FLAGS_alpha_heatmap,
            FLAGS_part_to_show, FLAGS_model_folder, heatMapTypes, heatMapScaleMode, FLAGS_part_candidates,
            (float)FLAGS_render_threshold, FLAGS_number_people_max, FLAGS_maximize_positives, FLAGS_fps_max,
            FLAGS_prototxt_path, FLAGS_caffemodel_path, (float)FLAGS_upsampling_ratio, enableGoogleLogging};
        opWrapper.configure(wrapperStructPose);
        // Face configuration (use op::WrapperStructFace{} to disable it)
        const op::WrapperStructFace wrapperStructFace{
            FLAGS_face, faceDetector, faceNetInputSize,
            op::flagsToRenderMode(FLAGS_face_render, multipleView, FLAGS_render_pose),
            (float)FLAGS_face_alpha_pose, (float)FLAGS_face_alpha_heatmap, (float)FLAGS_face_render_threshold};
        opWrapper.configure(wrapperStructFace);
        // Hand configuration (use op::WrapperStructHand{} to disable it)
        const op::WrapperStructHand wrapperStructHand{
            FLAGS_hand, handDetector, handNetInputSize, FLAGS_hand_scale_number, (float)FLAGS_hand_scale_range,
            op::flagsToRenderMode(FLAGS_hand_render, multipleView, FLAGS_render_pose), (float)FLAGS_hand_alpha_pose,
            (float)FLAGS_hand_alpha_heatmap, (float)FLAGS_hand_render_threshold};
        opWrapper.configure(wrapperStructHand);
        // Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
        const op::WrapperStructExtra wrapperStructExtra{
            FLAGS_3d, FLAGS_3d_min_views, FLAGS_identification, FLAGS_tracking, FLAGS_ik_threads};
        opWrapper.configure(wrapperStructExtra);
        // Producer (use default to disable any input)
        const op::WrapperStructInput wrapperStructInput{
            producerType, producerString, FLAGS_frame_first, FLAGS_frame_step, FLAGS_frame_last,
            FLAGS_process_real_time, FLAGS_frame_flip, FLAGS_frame_rotate, FLAGS_frames_repeat,
            cameraSize, FLAGS_camera_parameter_path, FLAGS_frame_undistort, FLAGS_3d_views};
        opWrapper.configure(wrapperStructInput);
        // Output (comment or use default argument to disable any output)
        const op::WrapperStructOutput wrapperStructOutput{
            FLAGS_cli_verbose, FLAGS_write_keypoint, op::stringToDataFormat(FLAGS_write_keypoint_format),
            FLAGS_write_json, FLAGS_write_coco_json, FLAGS_write_coco_json_variants, FLAGS_write_coco_json_variant,
            FLAGS_write_images, FLAGS_write_images_format, FLAGS_write_video, FLAGS_write_video_fps,
            FLAGS_write_video_with_audio, FLAGS_write_heatmaps, FLAGS_write_heatmaps_format, FLAGS_write_video_3d,
            FLAGS_write_video_adam, FLAGS_write_bvh, FLAGS_udp_host, FLAGS_udp_port};
        opWrapper.configure(wrapperStructOutput);
        // GUI (comment or use default argument to disable any visual output)
        if (!FLAGS_no_display) {
            const op::WrapperStructGui wrapperStructGui{
                op::flagsToDisplayMode(FLAGS_display, FLAGS_3d), !FLAGS_no_gui_verbose, FLAGS_fullscreen};
            opWrapper.configure(wrapperStructGui);
        }
        // Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
        if (FLAGS_disable_multi_thread)
            opWrapper.disableMultiThreading();
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}

int poseFusion()
{
    try
    {
        op::log("Starting PoseFusion...", op::Priority::High);
        const auto opTimer = op::getTimerInit();

        // OpenPose wrapper
        op::log("Configuring OpenPose...", op::Priority::High);
        op::Wrapper opWrapper;
        configureWrapper(opWrapper);

        // Start, run, and stop processing - exec() blocks this thread until OpenPose wrapper has finished
        op::log("Starting thread(s)...", op::Priority::High);
        opWrapper.exec();

        // Measuring total time
        op::printTime(opTimer, "PoseFusion successfully finished. Total time: ", " seconds.", op::Priority::High);

        // Return
        return 0;
    }
    catch (const std::exception& e)
    {
        return -1;
    }
}

int main(int argc, char *argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    // Run PoseFusion
    int ret = poseFusion();

    std::cout << "\nDone." << std::endl;
    return ret;
}
