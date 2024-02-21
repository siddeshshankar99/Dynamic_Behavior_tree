#include <ros/ros.h>
#include <behavior_tree.h>
#include <actions/ros_action.h>
#include <conditions/ros_condition.h>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "destructive_BT_R");

    try
    {
        int TickPeriod_milliseconds = 3000;

        // Declare and Allocate BT Nodes
        BT::ROSAction* action1 = new BT::ROSAction("Pick3_from_goal_R");
        BT::ROSAction* action3 = new BT::ROSAction("Back_to_home_R");
        BT::ROSCondition* condition1 = new BT::ROSCondition("button3_not_condition_R");
        BT::ROSAction* action4 = new BT::ROSAction("Place3_from_goal_R");       
        BT::ROSCondition* condition4 = new BT::ROSCondition("button3_condition_R");      
        BT::ROSCondition* condition11 = new BT::ROSCondition("button3_in_init_condition_R");
        
        
        // Set Action Duration
        //action1->set_time(5);

        // Create Sequence Node
        BT::SequenceNode* sequence1 = new BT::SequenceNode("seq1");
        BT::SequenceNode* sequence2 = new BT::SequenceNode("seq2");
        BT::SequenceNode* sequence3 = new BT::SequenceNode("seq3");
        
        // Create Fallback Node
        BT::FallbackNode* fallback1 = new BT::FallbackNode("fall1");
        BT::FallbackNode* fallback2 = new BT::FallbackNode("fall2");
        
        // Set Condition Value
        //condition1->set_boolean_value(true);
          
        //First Branch 
        // Build the Sequence 1
        sequence1->AddChild(condition1);
        sequence1->AddChild(action1);
        
        // Build the Fallback 1
        fallback1->AddChild(condition4);
        fallback1->AddChild(sequence1);
        
        // Build the Sequence 2 
        sequence2->AddChild(fallback1);
        sequence2->AddChild(action4);
        
        // Build the Fallback 2
        fallback2->AddChild(condition11); 
        fallback2->AddChild(sequence2);
        
        // Build the Sequence 3
        sequence3->AddChild(fallback2);
        sequence3->AddChild(action3);


        // Execute the Behavior Tree
        Execute(sequence3, TickPeriod_milliseconds);


    }
    catch (BT::BehaviorTreeException& Exception)
    {
        std::cout << Exception.what() << std::endl;
    }

    return 0;
}

