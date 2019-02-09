#!/usr/bin/env python

import heapq
import problem 
import rospy
from std_msgs.msg import String
import argparse
from collections import deque
import time

rospy.init_node("walk")
publisher = rospy.Publisher("/actions",String,queue_size =10)
parser = argparse.ArgumentParser()
parser.add_argument('-a',help = "Please mention algorithm to use. Default is BFS", metavar = 'bfs', action='store', dest='algorithm', default="bfs", type=str)

def stringifyState(state):
    return str(state.x) + str(state.y) + str(state.orientation);

def manhattanHeuristics(sourceState, destinationState):
    return abs(sourceState.x - destinationState.x) + abs(sourceState.y - destinationState.y);

def euclideanHeuristics(sourceState, destinationState):
    return ((sourceState.x - destinationState.x)**2) + ((sourceState.y - destinationState.y)**2);


def bfs():
    tic = time.clock();
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    print("init state", stringifyState(init_state));
    print("goal state", stringifyState(goal_state));
    possible_actions = problem.get_actions()

    action_list = [];
    paths = {};
    paths[stringifyState(init_state)] = action_list;

    if (problem.is_goal_state(init_state)):
        toc = time.clock();
        print(toc-tic);
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
                #print(stringifyState(current_state),stringifyState(nextstate), possible_action);
                path_e = current_path[:];
                path_e.append(possible_action);
                if(problem.is_goal_state(nextstate)):
                    print("goal found ");
                    #print(frontier);
                    #print(explored_states);
                    print(len(path_e));
                    toc = time.clock();
                    print(toc - tic);
                    return path_e;
                frontier.append(nextstate);
                paths[stringifyState(nextstate)] = path_e;

    print("goal not found");
    toc = time.clock();
    print(toc - tic);
    return [];

def ucs():
    tic = time.clock();
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    print("init state", stringifyState(init_state));
    print("goal state", stringifyState(goal_state));
    possible_actions = problem.get_actions()

    action_list = [];

    if (problem.is_goal_state(init_state)):
        return action_list;
    explored_states = {};
    frontier = [(0, init_state, action_list)];

    # to get the next state, cost for an action on state_x use:
    '''(nextstate, cost) = problem.get_successor(state, action)'''

    while frontier:
        [current_cost, current_state, current_path]= heapq.heappop(frontier);
        if stringifyState(current_state) in explored_states and explored_states[stringifyState(current_state)] < current_cost:
            #print("skipped", stringifyState(current_state));
            continue;
        if (problem.is_goal_state(current_state)):
            print("goal found ");
            print(len(current_path));
            toc = time.clock();
            print(toc - tic);
            return current_path;
        for possible_action in possible_actions:
            (nextstate, cost) = problem.get_successor(current_state, possible_action);
            if stringifyState(nextstate) not in explored_states and cost > 0:
                path_e = current_path[:];
                path_e.append(possible_action);
                totalCost = current_cost + cost;
                #print(stringifyState(current_state), stringifyState(nextstate), possible_action);
                heapq.heappush(frontier, (totalCost, nextstate, path_e));
        explored_states[stringifyState(current_state)] = current_cost;
    toc = time.clock();
    print(toc - tic);
    print("goal not found");
    return [];

def gbfs():
    tic = time.clock();
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    print("init state", stringifyState(init_state));
    print("goal state", stringifyState(goal_state));
    possible_actions = problem.get_actions()

    action_list = [];

    if (problem.is_goal_state(init_state)):
        return action_list;
    explored_states = {};
    frontier = [(manhattanHeuristics(init_state, goal_state), init_state, action_list)];

    # to get the next state, cost for an action on state_x use:
    '''(nextstate, cost) = problem.get_successor(state, action)'''

    while frontier:
        [current_cost, current_state, current_path] = heapq.heappop(frontier);
        if (problem.is_goal_state(current_state)):
            print("goal found ");
            print(len(current_path));
            toc = time.clock();
            print(toc - tic);
            return current_path;
        for possible_action in possible_actions:
            (nextstate, cost) = problem.get_successor(current_state, possible_action);
            if stringifyState(nextstate) not in explored_states and cost > 0:
                path_e = current_path[:];
                path_e.append(possible_action);
                # print(stringifyState(current_state), stringifyState(nextstate), possible_action);
                heapq.heappush(frontier, (manhattanHeuristics(nextstate, goal_state), nextstate, path_e));
        explored_states[stringifyState(current_state)] = manhattanHeuristics(nextstate, goal_state);
    toc = time.clock();
    print(toc - tic);
    print("goal not found");
    return [];

def astar():
    tic = time.clock();
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    print("init state", stringifyState(init_state));
    print("goal state", stringifyState(goal_state));
    possible_actions = problem.get_actions()

    action_list = [];

    if (problem.is_goal_state(init_state)):
        return action_list;
    explored_states = {};
    frontier = [(0+manhattanHeuristics(init_state, goal_state), init_state, action_list)];

    # to get the next state, cost for an action on state_x use:
    '''(nextstate, cost) = problem.get_successor(state, action)'''

    while frontier:
        [current_cost, current_state, current_path] = heapq.heappop(frontier);
        if stringifyState(current_state) in explored_states and explored_states[
            stringifyState(current_state)] < current_cost:
            # print("skipped", stringifyState(current_state));
            continue;
        if (problem.is_goal_state(current_state)):
            print("goal found ");
            print(len(current_path));
            toc = time.clock();
            print(toc - tic);
            return current_path;
        for possible_action in possible_actions:
            (nextstate, cost) = problem.get_successor(current_state, possible_action);
            if stringifyState(nextstate) not in explored_states and cost > 0:
                path_e = current_path[:];
                path_e.append(possible_action);
                totalCost = current_cost + cost + manhattanHeuristics(nextstate, goal_state);
                # print(stringifyState(current_state), stringifyState(nextstate), possible_action);
                heapq.heappush(frontier, (totalCost, nextstate, path_e));
        explored_states[stringifyState(current_state)] = current_cost;
    toc = time.clock();
    print(toc - tic);
    print("goal not found");
    return [];


   

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
