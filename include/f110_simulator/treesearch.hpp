#pragma once

#include "f110_simulator/car_state.hpp"
#include <array>
#include <vector>


/*
 * TODO: Apply smart pointers?
 *        Node evaluation method
 *        Implement MCTS instead of BFS
 *        Simulation steps per iteration?
 *        Reward System?
 */


// speed, angle
typedef std::pair<double, double> ActionPair;

class ActionTree {

    public:
    // tree state
    racecar_simulator::CarState state;
    ActionPair actions;
    double score;
    int visits;
    bool terminal;

    // multi directional
    ActionTree *parent;
    std::vector<ActionTree*> children;

    ActionTree(ActionTree *p, racecar_simulator::CarState s, ActionPair ap);
    ~ActionTree();

    double getScore();
    void setChildren(std::vector<ActionTree*> children);
    void propagateToRoot(double score);
    ActionTree* explore();
    bool isTerminal();
};


