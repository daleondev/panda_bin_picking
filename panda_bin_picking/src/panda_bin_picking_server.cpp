#include "franka_panda.h"
#include "visualizer.h"

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <thread>

#define NODE_NAME               "panda_bin_picking_server"

#define SHUTDOWN_TOPIC          "/panda_bin_picking/shutdown"

#define CAPTURE_SERVICE         "/panda_bin_picking/capture"
#define DETECT_SERVICE          "/panda_bin_picking/detect"
#define PLAN_APPROACH_SERVICE   "/panda_bin_picking/plan_approach"
#define APPROACH_SERVICE        "/panda_bin_picking/approach"
#define PLAN_PICK_SERVICE       "/panda_bin_picking/plan_pick"
#define PICK_SERVICE            "/panda_bin_picking/pick"
#define PLAN_LIFT_SERVICE       "/panda_bin_picking/plan_lift"
#define LIFT_SERVICE            "/panda_bin_picking/lift"
#define PLAN_LIFT_ALT_SERVICE   "/panda_bin_picking/plan_lift_alt"
#define LIFT_ALT_SERVICE        "/panda_bin_picking/lift_alt"
#define PLAN_PLACE_SERVICE      "/panda_bin_picking/plan_place"
#define PLACE_SERVICE           "/panda_bin_picking/place"

#define ADVERTISE_TRIGGER       advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>
#define BIND_CALLBACK(cb)       std::bind(cb, std::placeholders::_1, std::placeholders::_2, std::ref(franka_panda))

/**
 * Callback for capture service
 * 
 * @param[in] request the service request
 * @param[out] response the service response
 * @param[in] franka_panda the robot abstraction
 * @return bool success
 */
static bool captureCallback     (const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda);
/**
 * Callback for detect service
 * 
 * @param[in] request the service request
 * @param[out] response the service response
 * @param[in] franka_panda the robot abstraction
 * @return bool success
 */
static bool detectCallback      (const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda);
/**
 * Callback for plan approach service
 * 
 * @param[in] request the service request
 * @param[out] response the service response
 * @param[in] franka_panda the robot abstraction
 * @return bool success
 */
static bool planApproachCallback(const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda);
/**
 * Callback for approach service
 * 
 * @param[in] request the service request
 * @param[out] response the service response
 * @param[in] franka_panda the robot abstraction
 * @return bool success
 */
static bool approachCallback    (const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda);
/**
 * Callback for plan pick service
 * 
 * @param[in] request the service request
 * @param[out] response the service response
 * @param[in] franka_panda the robot abstraction
 * @return bool success
 */
static bool planPickCallback    (const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda);
/**
 * Callback for pick service
 * 
 * @param[in] request the service request
 * @param[out] response the service response
 * @param[in] franka_panda the robot abstraction
 * @return bool success
 */
static bool pickCallback        (const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda);
/**
 * Callback for plan lift service
 * 
 * @param[in] request the service request
 * @param[out] response the service response
 * @param[in] franka_panda the robot abstraction
 * @return bool success
 */
static bool planLiftCallback    (const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda);
/**
 * Callback for lift service
 * 
 * @param[in] request the service request
 * @param[out] response the service response
 * @param[in] franka_panda the robot abstraction
 * @return bool success
 */
static bool liftCallback        (const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda);
/**
 * Callback for plan lift alt service
 * 
 * @param[in] request the service request
 * @param[out] response the service response
 * @param[in] franka_panda the robot abstraction
 * @return bool success
 */
static bool planLiftAltCallback (const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda);
/**
 * Callback for lift alt service
 * 
 * @param[in] request the service request
 * @param[out] response the service response
 * @param[in] franka_panda the robot abstraction
 * @return bool success
 */
static bool liftAltCallback     (const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda);
/**
 * Callback for plan place service
 * 
 * @param[in] request the service request
 * @param[out] response the service response
 * @param[in] franka_panda the robot abstraction
 * @return bool success
 */
static bool planPlaceCallback   (const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda);
/**
 * Callback for place service
 * 
 * @param[in] request the service request
 * @param[out] response the service response
 * @param[in] franka_panda the robot abstraction
 * @return bool success
 */
static bool placeCallback       (const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda);

/**
 * Main function
 * 
 * Initializes the node, visualization, and robot. Advertises services and listens for requests
 * 
 * @param[in] argc number of cmd line arguments
 * @param[in] argv array containing cmd line arguments
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle node_handle;
    ros::AsyncSpinner async_spinner(std::thread::hardware_concurrency());
    async_spinner.start();

    // initialize visualization
    Visualizer::init(node_handle);

    // initialize robot abstaction
    FrankaPanda franka_panda(node_handle);

    // advertise services
    ros::ServiceServer srvs[] = {
        node_handle.ADVERTISE_TRIGGER(CAPTURE_SERVICE, BIND_CALLBACK(captureCallback)),
        node_handle.ADVERTISE_TRIGGER(DETECT_SERVICE, BIND_CALLBACK(detectCallback)),
        node_handle.ADVERTISE_TRIGGER(PLAN_APPROACH_SERVICE, BIND_CALLBACK(planApproachCallback)),
        node_handle.ADVERTISE_TRIGGER(APPROACH_SERVICE, BIND_CALLBACK(approachCallback)),
        node_handle.ADVERTISE_TRIGGER(PLAN_PICK_SERVICE, BIND_CALLBACK(planPickCallback)),
        node_handle.ADVERTISE_TRIGGER(PICK_SERVICE, BIND_CALLBACK(pickCallback)),
        node_handle.ADVERTISE_TRIGGER(PLAN_LIFT_SERVICE, BIND_CALLBACK(planLiftCallback)),
        node_handle.ADVERTISE_TRIGGER(LIFT_SERVICE, BIND_CALLBACK(liftCallback)),
        node_handle.ADVERTISE_TRIGGER(PLAN_LIFT_ALT_SERVICE, BIND_CALLBACK(planLiftAltCallback)),
        node_handle.ADVERTISE_TRIGGER(LIFT_ALT_SERVICE, BIND_CALLBACK(liftAltCallback)),
        node_handle.ADVERTISE_TRIGGER(PLAN_PLACE_SERVICE, BIND_CALLBACK(planPlaceCallback)),
        node_handle.ADVERTISE_TRIGGER(PLACE_SERVICE, BIND_CALLBACK(placeCallback)),
    };

    // shutdown node when msg on shutdown-topic arrives
    ros::topic::waitForMessage<std_msgs::Empty>(SHUTDOWN_TOPIC);
    ros::shutdown();
    return 0;
}

static bool captureCallback(const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda)
{
    response.success = franka_panda.capture();
    return true;
}

static bool detectCallback(const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda)
{
    response.success = franka_panda.detect();
    return true;
}

static bool planApproachCallback(const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda)
{
    Visualizer::clear();
    response.success = franka_panda.planApproach();
    return true;
}

static bool approachCallback(const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda)
{
    response.success = franka_panda.approach();
    return true;
}

static bool planPickCallback(const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda)
{
    Visualizer::clear();
    response.success = franka_panda.planPick();
    return true;
}

static bool pickCallback(const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda)
{
    response.success = franka_panda.pick();
    return true;
}

static bool planLiftCallback(const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda)
{
    Visualizer::clear();
    response.success = franka_panda.planLift();
    return true;
}

static bool liftCallback(const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda)
{
    response.success = franka_panda.lift();
    return true;
}

static bool planLiftAltCallback(const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda)
{
    Visualizer::clear();
    response.success = franka_panda.planLiftAlt();
    return true;
}

static bool liftAltCallback(const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda)
{
    response.success = franka_panda.liftAlt();
    return true;
}

static bool planPlaceCallback(const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda)
{
    Visualizer::clear();
    response.success = franka_panda.planPlace();
    return true;
}

static bool placeCallback(const std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response, FrankaPanda& franka_panda)
{
    response.success = franka_panda.place();
    Visualizer::clear();
    return true;
}
