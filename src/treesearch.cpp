#include "f110_simulator/treesearch.hpp"
#include <vector>

ActionTree::ActionTree(ActionTree *p, racecar_simulator::CarState s, ActionPair ap)
{
    score    = 0;
    visits   = 0;
    state    = s;
    parent   = p;
    actions  = ap;
    children = std::vector<ActionTree*>();
    terminal = false;
}

ActionTree::~ActionTree()
{
    /*
     * Child pointers are the only dynamically stored, so they need
     * to be disposed first.
     */

    for(auto ch: children)
    {
        delete ch;
    }
}

double ActionTree::getScore()
{
    /*
    * Change this or remove it
    */

    return score/visits;
}

void ActionTree::setChildren(std::vector<ActionTree*> children)
{
    /*
    * Given a vector of ActionTree pointers, take ownership of them.
    */

    for(auto chld: children)
    {
        children.push_back(chld);
    }
}

void ActionTree::propagateToRoot(double rollout_reward)
{
    /*
     * Given a node("this"), walk up to the root node and update visits
     * and the rollout reward(scoring of "this" node).
     */

    if(parent != NULL)
    {
        visits += 1;
        parent->propagateToRoot(rollout_reward);
    }
}

ActionTree* ActionTree::explore()
{
    /*
     * Walk down the tree to find the best node to work from..
     */

    if(children.size() == 0)
    {
        return this;
    }
    else
    {
        ActionTree* next_step = children[0];
        double      uct_score = children[0]->getScore();
        for(size_t i=1; i < children.size(); ++i)
        {
            double t_score = children[i]->getScore();
            if(t_score > uct_score && !children[i]->isTerminal())
            {
                uct_score = t_score;
                next_step = children[i];
            }
        }

        // run recursivly down the tree
        next_step->explore();
    }
}

bool ActionTree::isTerminal()
{
    return terminal;
}
