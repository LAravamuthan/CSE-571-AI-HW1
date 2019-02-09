#!/usr/bin/env python

import heapq
import problem 
import rospy
from std_msgs.msg import String
import argparse
from collections import deque

rospy.init_node("walk")
publisher = rospy.Publisher("/actions",String,queue_size =10)
parser = argparse.ArgumentParser()
parser.add_argument('-a',help = "Please mention algorithm to use. Default is BFS", metavar = 'bfs', action='store', dest='algorithm', default="bfs", type=str)

def stringifyState(state):
    return str(state.x) + str(state.y) + str(state.orientation);

def contains(heap, value):
    if len([(x, y) for x, y in heap if y == value]) > 0:
        return True
    return False;

def costLessOverHead(heap, value, cost, paths, path_e):
    newHeap = [i for i in heap if i[1] != value or i[0]< cost];
    if len(newHeap) < len(heap):
        print("frontier condition happened");
        newHeap.append((cost, value));
        paths[stringifyState(value)] = [path_e, cost];
    heapq.heapify(newHeap);
    return newHeap;

def bfs():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    print("init state", stringifyState(init_state));
    print("goal state", stringifyState(goal_state));
    possible_actions = problem.get_actions()

    action_list = [];
    paths = {};
    paths[stringifyState(init_state)] = action_list;

    if (problem.is_goal_state(init_state)):
        return paths[stringifyState(init_state)];
    explored_states = [];
    frontier = deque();
    frontier.append(init_state);

    #to get the next state, cost for an action on state_x use:
    '''(nextstate, cost) = problem.get_successor(state, action)'''

    while frontier:
        current_state = frontier.popleft();
        current_path = paths[stringifyState(current_state)];
        explored_states.append(current_state);
        for possible_action in possible_actions:
            (nextstate, cost) = problem.get_successor(current_state,possible_action);
            if nextstate not in explored_states and nextstate not in frontier and cost > 0:
                print(stringifyState(current_state),stringifyState(nextstate), possible_action);
                path_e = current_path[:];
                path_e.append(possible_action);
                if(problem.is_goal_state(nextstate)):
                    print("goal found ");
                    #print(frontier);
                    #print(explored_states);
                    return path_e;
                frontier.append(nextstate);
                paths[stringifyState(nextstate)] = path_e;

    print("goal not found");
    return [];

def ucs():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    print("init state", stringifyState(init_state));
    print("goal state", stringifyState(goal_state));
    possible_actions = problem.get_actions()

    action_list = [];
    paths = {};
    paths[stringifyState(init_state)] = [action_list, 0];

    if (problem.is_goal_state(init_state)):
        return paths[stringifyState(init_state)];
    explored_states = [];
    frontier = [];
    heapq.heappush(frontier, (0, init_state));

    # to get the next state, cost for an action on state_x use:
    '''(nextstate, cost) = problem.get_successor(state, action)'''

    while frontier:
        current_state = heapq.heappop(frontier)[1];
        [current_path, current_cost] = paths[stringifyState(current_state)];
        if (problem.is_goal_state(current_state)):
            print("goal found ");
            return current_path;
        explored_states.append(current_state);
        for possible_action in possible_actions:
            (nextstate, cost) = problem.get_successor(current_state, possible_action);
            path_e = current_path[:];
            path_e.append(possible_action);
            totalCost = current_cost + cost;
            if nextstate not in explored_states and not contains(frontier, nextstate) and cost > 0:
                print(stringifyState(current_state), stringifyState(nextstate), possible_action);
                heapq.heappush(frontier, (totalCost, nextstate));
                paths[stringifyState(nextstate)] = [path_e,totalCost];
            frontier = costLessOverHead(frontier, nextstate, totalCost, paths, path_e);
    print("goal not found");
    return [];

def gbfs():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions() 
    action_list = []

    #to get the next state, cost for an action on state_x use:


    '''
     (nextstate, cost) = problem.get_successor(state, action)
    YOUR CODE HERE
    '''


    return action_list

def astar():
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions() 
    action_list = []

    #to get the next state, cost for an action on state_x use:


    '''
    YOUR CODE HERE
    (nextstate, cost) = problem.get_successor(state, action)
    '''


    return action_list


   

 # to execute a plan action_list = <list of actions>, use:
def exec_action_list(action_list):
    plan_str = '_'.join(action for action in action_list)
    publisher.publish(String(data = plan_str))

if __name__ == "__main__":
    args = parser.parse_args()
    algorithm = globals().get(args.algorithm)
    if algorithm is None:
        print "Incorrect Algorithm name."
        exit(1)
    actions = algorithm();
    print(actions);
    exec_action_list(actions);
