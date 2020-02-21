#include <ros/ros.h>

// interactive marker
#include <interactive_markers/interactive_marker_server.h>

#include <tf2/impl/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32MultiArray.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include "f110_simulator/pose_2d.hpp"
#include "f110_simulator/ackermann_kinematics.hpp"
#include "f110_simulator/scan_simulator_2d.hpp"

#include "f110_simulator/car_state.hpp"
#include "f110_simulator/car_params.hpp"
#include "f110_simulator/ks_kinematics.hpp"
#include "f110_simulator/st_kinematics.hpp"
#include "f110_simulator/precompute.hpp"

#include <iostream>
#include <math.h>
#include <array>
#include <vector>


/*
 * TODO: Apply smart pointers?
 *        Node evaluation method
 *        Implement MCTS instead of BFS
 *
 *        Move tree stuff into a seperate .cpp .h files
 */


using namespace racecar_simulator;

// speed, angle
typedef std::pair<double, double> ActionPair;

class ActionTree {

    public:

    // tree state
    CarState state;
    ActionPair actions;
    double score;
    int visits;
    bool terminal;

    // multi directional
    ActionTree *parent;
    std::vector<ActionTree*> children;

    ActionTree(ActionTree *p, CarState s, ActionPair ap)
    {
        score = 0;
        visits = 1;
        state = s;
        parent = p;
        actions = ap;
        children = std::vector<ActionTree*>();
        terminal = false;
    }

    ~ActionTree()
    {
        for(auto child: children)
            delete child;
    }

    double getScore()
    {
        return score/visits;
    }

    void setChildren(std::vector<ActionTree*> children)
    {
        for(auto child: children)
            this->children.push_back(child);
    }

    void propagateToRoot(double score)
    {
        if(parent != NULL)
        {
            this->score += score;
            parent->propagateToRoot(score);
        }
    }

    ActionTree* explore()
    {
        // check if leaf
        if(children.size() == 0)
        {
            return this;
        }
        else
        {
            ActionTree* next_step = children[0];
            double      cur_score = children[0]->getScore();
            for(size_t i=1; i < children.size(); ++i)
            {
                // select child with better score, and not terminal state
                if(children[i]->getScore() > cur_score && !children[i]->terminal)
                {
                    cur_score = children[i]->score;
                    next_step = children[i];
                    break;
                }
            }
            next_step->visits += 1;
            next_step->explore();
        }
    }
};


