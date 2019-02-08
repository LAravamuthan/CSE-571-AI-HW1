#!/usr/bin/env python

import heapq
import problem 
import rospy
from std_msgs.msg import String
import argparse
import Queue as queue

publisher = rospy.Publisher("/actions",String,queue_size =10)
parser = argparse.ArgumentParser()
parser.add_argument('-a',help = "Please mention algorithm to use. Default is BFS", metavar = 'bfs', action='store', dest='algorithm', default="bfs", type=str)



def bfs():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions()
    frontier = queue.Queue();
    action_list = [];
    paths = {};
    paths[str(init_state.x) + str(init_state.y) + str(init_state.orientation)] = action_list;

    if (problem.is_goal_state(init_state)):
        return paths[init_state];
    explored_states = set();
    frontier.put(init_state);

    #to get the next state, cost for an action on state_x use:
    '''(nextstate, cost) = problem.get_successor(state, action)'''

    while frontier.qsize()>0:
        current_state = frontier.get();
        current_path = paths[str(current_state.x) + str(current_state.y) + str(current_state.orientation)];
        explored_states.add(current_state);
        for possible_action in possible_actions:
            (nextstate, cost) = problem.get_successor(possible_action, current_state);
            if nextstate not in explored_states and nextstate not in frontier:
                path_e = current_path;
                path_e.append(possible_action);
                if(problem.is_goal_state(nextstate)):
                    return path_e;
                frontier.put(nextstate);
                paths[str(nextstate.x) + str(nextstate.y) + str(nextstate.orientation)] = path_e;


    return action_list

def ucs():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions() 
    action_list = []

    #to get the next state, cost for an action on state_x use:
    (nextstate, cost) = problem.get_successor(state, action)

    '''
    YOUR CODE HERE
    '''


    return action_list

def gbfs():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions() 
    action_list = []

    #to get the next state, cost for an action on state_x use:
    (nextstate, cost) = problem.get_successor(state, action)

    '''
    YOUR CODE HERE
    '''


    return action_list

def astar():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions() 
    action_list = []

    #to get the next state, cost for an action on state_x use:
    (nextstate, cost) = problem.get_successor(state, action)

    '''
    YOUR CODE HERE
    '''


    return action_list


   

 # to execute a plan action_list = <list of actions>, use:
def exec_action_list(self, action_list):
    plan_str = '_'.join(action for action in action_list)
    publisher.publish(String(data = plan_str))

if __name__ == "__main__":
    args = parser.parse_args()
    algorithm = globals().get(args.algorithm)
    if algorithm is None:
        print "Incorrect Algorithm name."
        exit(1)
    actions = algorithm()
    exec_action_list(actions)



