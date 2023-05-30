#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#define NODE_NAME "panda_bin_picking_client"

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

/**
 * Throw error
 * 
 * @param[in] msg error description
 */
static void error(const char* msg);
/**
 * Statemachine
 * 
 * @param[in] node_handle the ros node handle of the current node
 */
static void stateMachine(ros::NodeHandle& node_handle);

/**
 * Main function
 * 
 * Initializes the node an starts the state machine
 * 
 * @param[in] argc number of cmd line arguments
 * @param[in] argv array containing cmd line arguments
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle node_handle;
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  stateMachine(node_handle);

  ros::shutdown();
  return 0;
}

static void stateMachine(ros::NodeHandle& node_handle)
{
  // all possible state machine states
  enum class State : uint8_t {
    Home          = 0,
    Capture       = 1,
    Detect        = 2,
    PlanApproach  = 3,
    Approach      = 4,
    PlanPick      = 5,
    Pick          = 6,
    PlanLift      = 7,
    Lift          = 8,
    PlanLiftAlt   = 9,
    LiftAlt       = 10,
    PlanPlace     = 11,
    Place         = 12,
    Finished      = 13
  };

  // each state mapped to its description
  std::map<State, std::string> state_to_desc = {
    {State::Capture,      "capture pointclouds"},
    {State::Detect,       "detect grasp poses"},
    {State::PlanApproach, "plan approach path"},
    {State::Approach,     "execute approach path"},
    {State::PlanPick,     "plan pick path"},
    {State::Pick,         "execute pick path"},
    {State::PlanLift,     "plan lift path"},
    {State::Lift,         "execute lift path"},
    {State::PlanLiftAlt,  "plan alternative lift path"},
    {State::LiftAlt,      "execute alternative lift path"},
    {State::PlanPlace,    "plan place path"},
    {State::Place,        "execute place path"},
    {State::Finished,     "finish"},
  };

  // start in home state
  State state = State::Home;

  std_srvs::Trigger srv;

  // actual state machine
  while (true) {
    switch (state) {
      case State::Home:
      {
        state = State::Capture;
        break;
      }
      case State::Capture:
      {
        ros::service::waitForService(CAPTURE_SERVICE);
        ros::service::call(CAPTURE_SERVICE, srv);
        if (!srv.response.success) {
          error("capture failed");
        }
        state = State::Detect;
        break;
      }
      case State::Detect:
      {
        ros::service::call(DETECT_SERVICE, srv);
        if (!srv.response.success) {
          ROS_INFO("no objects left to grasp");
          ros::shutdown();
          return;
        }
        state = State::PlanApproach;
        break;
      }
      case State::PlanApproach:
      {
        ros::service::call(PLAN_APPROACH_SERVICE, srv);
        if (!srv.response.success) {
          error("approach failed");
        }
        state = State::Approach;
        break;
      }
      case State::Approach:
      {
        ros::service::call(APPROACH_SERVICE, srv);
        if (!srv.response.success) {
          state = State::PlanApproach;
        }
        else {
          state = State::PlanPick;
        }
        break;
      }
      case State::PlanPick:
      {
        ros::service::call(PLAN_PICK_SERVICE, srv);
        if (!srv.response.success) {
          state = State::PlanApproach;
        }
        else {
          state = State::Pick;
        }
        break;
      }
      case State::Pick:
      {
        ros::service::call(PICK_SERVICE, srv);
        if (!srv.response.success) {
          error("pick failed");
        }
        else {
          state = State::PlanLift;
        }
        break;
      }
      case State::PlanLift:
      {
        ros::service::call(PLAN_LIFT_SERVICE, srv);
        if (!srv.response.success) {
          //state = State::PlanLiftAlt;
          error("plan lift failed");
        }
        else {
          state = State::Lift;
        }
        break;
      }
      case State::Lift:
      {
        ros::service::call(LIFT_SERVICE, srv);
        if (!srv.response.success) {
          error("lift failed");
        }
        else {
          state = State::PlanPlace;
        }
        break;
      }
      case State::PlanLiftAlt:
      {
        ros::service::call(PLAN_LIFT_ALT_SERVICE, srv);
        if (!srv.response.success) {
          error("plan lift alt failed");
        }
        else {
          state = State::LiftAlt;
        }
        break;
      }
      case State::LiftAlt:
      {
        ros::service::call(LIFT_ALT_SERVICE, srv);
        if (!srv.response.success) {
          error("lift alt failed");
        }
        else {
          state = State::PlanPlace;
        }
        break;
      }
      case State::PlanPlace:
      {
        ros::service::call(PLAN_PLACE_SERVICE, srv);
        if (!srv.response.success) {
          error("plan place failed");
        }
        else {
          state = State::Place;
        }
        break;
      }
      case State::Place:
      {
        ros::service::call(PLACE_SERVICE, srv);
        if (!srv.response.success) {
          error("place failed");
        }
        else {
          state = State::Finished;
        }
        break;
      }
      case State::Finished:
      {
        state = State::Capture;
        break;
      }
    }

    // wait for enter to change state (quit if input is 'q')
    ROS_INFO("Press enter to %s", state_to_desc.at(state).c_str());
    if(std::cin.get() == 'q')
      error("aborted");
  }
}

static void error(const char* msg)
{
  // print error description
  ROS_ERROR(msg);

  // shutdown node
  ros::shutdown();
  exit(1);
}