#include <ros/ros.h>
#include <behavior_tree.h>
#include <actions/ros_action.h>
#include <conditions/ros_condition.h>
#include <iostream>
#include <set>
#include <string>
#include <dot_bt.h>
#include <queue>
#include <map>
#include <unordered_map>
#include <vector>
#include <thread>
#include <algorithm>


void BT::ControlNode::RemoveChild(TreeNode* child)
{
    auto& children = getChildrenNodes();
    auto it = std::find(children.begin(), children.end(), child);
    
    if (it != children.end())
    {
        size_t index = std::distance(children.begin(), it);
        
        // Remove the child from children_nodes_
        children_nodes_.erase(it);
        
        //Also remove the corresponding status from children_states_
        auto& childStates = getChildrenStates();
        if(index < childStates.size())
        {
            children_states_.erase(childStates.begin() + index);
        }
        
        child->set_has_parent(false);
        
    }
    else 
    {
        throw BehaviorTreeException("Child not found. Cannot remove the child.");
    }
    std::cout << "Remove function has been finished" << std::endl;
}

void BT::ControlNode::InsertChildAtPosition(unsigned int position, TreeNode* child)
{
    if (position <= children_nodes_.size())
    {
        children_nodes_.insert(children_nodes_.begin() + position, child);
        // You can initialize the child's state to IDLE 
        children_states_.insert(children_states_.begin() + position, BT::IDLE);  
    }
    else 
    {
        throw BehaviorTreeException("Cannot insert child. Invalid position.");
    }
}


void Substitute(BT::ControlNode* root, BT::TreeNode* failedCondition, BT::ControlNode* newFallbackNode) 
{
    // Iterate over all children of the root
    for (unsigned int i = 0; i < root->GetChildrenNumber(); i++) 
    {
        // If we find the failedCondition as a direct child of root
        if (root->GetChildren()[i] == failedCondition) 
        {
            std::cout << "Getting ready to remove condition" << std::endl;
            root->RemoveChild(failedCondition);
            std::cout << "Removed the condition" << std::endl;

            // Add the newFallbackNode at the exact position of the removed child
            root->InsertChildAtPosition(i, newFallbackNode);

            std::cout << "Added the condition to the new fallback" << std::endl;
            return;  // Stop searching once we've made the substitution
        }
        // If the child is another control node, we recursively search inside it
        else if (BT::ControlNode* controlChild = dynamic_cast<BT::ControlNode*>(root->GetChildren()[i])) 
        {
            Substitute(controlChild, failedCondition, newFallbackNode);
        }
    }
    std::cout << "Substitute function successful" << std::endl;
}



std::set<std::string> ExpandedNodes;



BT::ConditionNode* GetConditionToExpand(BT::ControlNode* root)
{
    std::queue<BT::TreeNode*> queue;
    std::set<BT::TreeNode*> visited;

    // Start BFS with the root node
    queue.push(root);
    visited.insert(root);

    while (!queue.empty())
    {
        BT::TreeNode* current = queue.front();
        queue.pop();

        // If the current node is a condition and its status is FAILURE and it's not already expanded
        BT::ConditionNode* conditionNode = dynamic_cast<BT::ConditionNode*>(current);
        if(conditionNode && 
   (conditionNode->get_status() == BT::FAILURE || conditionNode->get_status() != BT::RUNNING) && 
   ExpandedNodes.find(conditionNode->get_name()) == ExpandedNodes.end())
        {
            ExpandedNodes.insert(conditionNode->get_name());
            return conditionNode;
        }

        // Add child nodes to the queue 
        BT::ControlNode* controlNode = dynamic_cast<BT::ControlNode*>(current);
        if (controlNode)
        {
            for (BT::TreeNode* child : controlNode->getChildrenNodes())
            {
                if (visited.find(child) == visited.end())
                {
                    queue.push(child);
                    visited.insert(child);
                }
            }
        }
    }
    
    return nullptr;  // If no condition to expand is found, return nullptr
}



std::string GetAction(BT::ConditionNode* condition)
{
    // Create a map of conditions to actions
    static std::map<std::string, std::string> conditionToActionMap = {
        {"button1_in_goal_condition_L", "Place1_L"},
        {"button2_in_goal_condition_L", "Place2_L"},
        {"button3_in_goal_condition_L", "Place3_L"},
        {"goal1_obstruct", "PlaceO_L"},
        {"button1_condition_L", "Pick1_L"},
        {"button2_condition_L", "Pick2_L"},
        {"button3_condition_L", "Pick3_L"},
        {"buttonO_condition_L", "PickO_L"}
    };

    // Use the map to get the corresponding action for a given condition
    auto it = conditionToActionMap.find(condition->get_name());
    if (it != conditionToActionMap.end())
    {
        return it->second;  // Return the corresponding action
    }

    // Default: If the condition isn't recognized, return an empty string or a default action
    return "";
}




// Define a map to hold the association between actions and their preconditions
std::unordered_map<std::string, std::vector<std::string>> actionToPreconditionsMap;

// Populate the map with actions and their associated preconditions
void InitializeActionPreconditionsMap() {
    
    actionToPreconditionsMap["Pick1_L"] = {"goal1_obstruct", "button1_not_condition_L"};
    actionToPreconditionsMap["Pick2_L"] = {"goal2_obstruct", "button2_not_condition_L"};
    actionToPreconditionsMap["Pick3_L"] = {"goal3_obstruct", "button3_not_condition_L"};
    actionToPreconditionsMap["Place1_L"] = {"button1_condition_L"};
    actionToPreconditionsMap["Place2_L"] = {"button2_condition_L"};
    actionToPreconditionsMap["Place3_L"] = {"button3_condition_L"};
    actionToPreconditionsMap["PickO_L"] = {"buttonO_not_condition_L"};
    actionToPreconditionsMap["PlaceO_L"] = {"buttonO_condition_L"};    

}

// Function to retrieve the preconditions for a given action
std::vector<std::string> GetPreconditionsForAction(const std::string& actionName) {
    auto it = actionToPreconditionsMap.find(actionName);
    if (it != actionToPreconditionsMap.end()) {
        return it->second;
    } else {
        // Return an empty vector if the action name is not found
        return {};
    }
}


std::pair<BT::ControlNode*, BT::FallbackNode*> ExpandTree(BT::ControlNode* sequence, BT::ConditionNode* cf) {
    
    auto ABT = GetAction(cf);

    // Check if the action is empty (no preconditions)
    if (ABT.empty() || ABT == "") {
        std::cout << "Skipping expansion for condition: " << cf->get_name() << " due to empty action." << std::endl;
        return {sequence, nullptr};  // Return the unchanged sequence and no outerFallback
    }
    
    //print the action for the failed condition
    std::cout << "Action for the failed condition: " << ABT << std::endl;

    // Create the outer fallback node
    BT::FallbackNode* outerFallback = new BT::FallbackNode("outer_fallback_for_" + cf->get_name());
    outerFallback->AddChild(new BT::ROSCondition(cf->get_name()));

    
    // New sequence node for this action
    BT::SequenceNode* BTseq = new BT::SequenceNode("sequence_for_" + ABT);
    
    // Fetch the preconditions for the specified action
    std::vector<std::string> preconditions = GetPreconditionsForAction(ABT);
    
    //print the precondition for the action
    std::cout << "The precondition for the Action: ";
    for (const auto& precondition : preconditions) {
         std::cout << precondition << " ";
    }
    std::cout << std::endl;

    // Add preconditions of the action to the sequence
    for (const auto& precondition : preconditions) {
        BTseq->AddChild(new BT::ROSCondition(precondition));
        std::cout << "Precondition's sequence is running" << std::endl;
    }

    // Add the action itself to the sequence
    BTseq->AddChild(new BT::ROSAction(ABT));
    std::cout << "Action's sequence is running" << std::endl;

    outerFallback->AddChild(BTseq);  // Add the sequence node
    std::cout << "inner fallback is running" << std::endl;
    
    //Substitue function 
    std::cout << "Substitute is getting ready" << std::endl;
    Substitute(sequence, cf, outerFallback);
    //Substitue function is working
    std::cout << "Substitute is working" << std::endl;


    return {sequence, outerFallback};
}

BT::TreeNode* FindChildByName(BT::ControlNode* node, const std::string& name)
{
    for (auto child : node->GetChildren())
    {
        std::cout << "Checking child: " << child->get_name() << std::endl;  
        if (child->get_name() == name)
        {
            std::cout << "Found child: " << name << std::endl;  
            return child;
        }
    }
    std::cout << "Child not found: " << name << std::endl; 
    return nullptr;
}

static bool drawing_started = false;
     
bool Execute_h(BT::ControlNode* root, int TickPeriod_milliseconds)
{

    // Vector to keep track of threads
    std::vector<std::thread> threads;

    if (!drawing_started)
    {
        std::cout << "Start Drawing!" << std::endl;
        drawing_started = true;

        threads.push_back(std::thread(&drawTree, root));
        
        std::cout << "New thread is starting!" << std::endl;
        // Start the new ticking thread
        BT::DotBt dotbt(root);
        threads.push_back(std::thread(&BT::DotBt::publish, dotbt));
        
    }
    

    // Give a short delay to allow the new ticking thread to start running
    std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 50 milliseconds delay (can adjust as needed)
  
    root->ResetColorState();
    int failureCounter = 0;  // Counter to track the number of failures
    while (ros::ok())
    {
        DEBUG_STDOUT("Ticking the root node !");
        std::this_thread::sleep_for(std::chrono::milliseconds(TickPeriod_milliseconds));
        root->Tick();
        std::cout << "New Thread started to tick" << std::endl;
        
        BT::ReturnStatus r = root->get_status();
        
        std::cout << "Return status: " << r << std::endl;

        if (r == BT::FAILURE)
        {
            failureCounter++;  // Increment the failure counter

            if (failureCounter >= 3)  // If the root has been ticked 3 times after a failure
            {
                break;  // Break out of the loop
            }
            
            BT::ConditionNode* Cf = GetConditionToExpand(root);
            if (Cf)
            {
                std::cout << "Expanding tree using condition: " << Cf->get_name() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(TickPeriod_milliseconds));
                
                auto [updated_BT, BT_new_subtree] = ExpandTree(root, Cf);  
                root = static_cast<BT::SequenceNode*>(updated_BT);
            
            }
            else
            {
                break;
            }
        
        }
           
        else if (r != BT::SUCCESS)
        {
            if (root->get_status() != BT::RUNNING)
            { 
                std::cout << "changing color" << std::endl;
                root->ResetColorState();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(TickPeriod_milliseconds));
        }
        else 
        {
        
            bool addedNewCondition = false;
            
            // Check the status of the nodes and add the new goal conditions
            BT::TreeNode* node_button1 = FindChildByName(root, "outer_fallback_for_button1_in_goal_condition_L");
            // If node_button1 is not found, try to find the node with the name "button1_in_goal_condition_L"
            if (!node_button1) {
                 node_button1 = FindChildByName(root, "button1_in_goal_condition_L");
            }
            std::cout << "Checking if button1_in_goal_condition is successful" << std::endl;
            if (node_button1 && node_button1->get_status() != BT::FAILURE )
            {
                BT::TreeNode* node_button2 = FindChildByName(root, "outer_fallback_for_button2_in_goal_condition_L");
                // If node_button2 is not found, try to find the node with the name "button2_in_goal_condition_L"
                if (!node_button2) {
                    node_button2 = FindChildByName(root, "button2_in_goal_condition_L");
                }
                std::cout << "Checking if button2_in_goal_condition is successful" << std::endl;
                if (!node_button2)  // Check if node_button2 doesn't already exist                
                {
                    std::cout << "Going to add button2_in_goal_condition" << std::endl;
                    root->AddChild(new BT::ROSCondition("button2_in_goal_condition_L"));
                    std::cout << "adding button2_in_goal_condition" << std::endl;
                    addedNewCondition = true;
                }
            }

            // Only run this part if no new condition was added in the first part
            if (!addedNewCondition)
            {
                BT::TreeNode* node_button2 = FindChildByName(root, "outer_fallback_for_button2_in_goal_condition_L");
                // If node_button2 is not found, try to find the node with the name "button2_in_goal_condition_L"
                if (!node_button2) {
                    node_button2 = FindChildByName(root, "button2_in_goal_condition_L");
                }
                std::cout << "Checking if button2_in_goal_condition is successful" << std::endl;
                if (node_button2 && node_button2->get_status() != BT::FAILURE)
                {
                    BT::TreeNode* node_button3 = FindChildByName(root, "outer_fallback_for_button3_in_goal_condition_L");
                    // If node_button3 is not found, try to find the node with the name "button3_in_goal_condition_L"
                    if (!node_button3) {
                        node_button3 = FindChildByName(root, "button3_in_goal_condition_L");
                    }
                    std::cout << "Checking if button3_in_goal_condition is successful" << std::endl;
                    if (!node_button3)  // Check if node_button3 doesn't already exist
                    {
                       root->AddChild(new BT::ROSCondition("button3_in_goal_condition_L"));
                       std::cout << "adding button3_in_goal_condition" << std::endl;
                       addedNewCondition = true;  // Though not necessary to set here, added for consistency
                    }
                }
            }


            // If you've added a new condition, reset the root's status to RUNNING
            if (addedNewCondition)
            {
                root->set_status(BT::RUNNING);
                continue;  // Continue to re-tick the root node
            }
            else
            {
                break;  // Exit the loop if no new condition was added and the root status is SUCCESS
            }
        }
    }

    // Join all threads before exiting the function
    for (auto& th : threads)
    {
        if (th.joinable())
        {
            th.join();
        }
    }
    // Return true if the root's status is not SUCCESS; otherwise, return false.
    return (root->get_status() != BT::SUCCESS);
}


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "Dynamic_behaviorTree_L");
    // Initialize the map
    InitializeActionPreconditionsMap();
    
    try
    {
    
        int TickPeriod_milliseconds = 3000;
        // Create the initial Behavior Tree (BT) with the goal conditions
        BT::SequenceNode* sequence1 = new BT::SequenceNode("root");
        
        //Initial Branch
        sequence1->AddChild(new BT::ROSCondition("button1_in_goal_condition_L"));
        
        bool continueExecution = true;
        while (continueExecution)
        {
             continueExecution = Execute_h(sequence1, TickPeriod_milliseconds);
        }
         
    }
    catch (BT::BehaviorTreeException& Exception)
    {
         std::cout << Exception.what() << std::endl;
    }   

    return 0;
}    
