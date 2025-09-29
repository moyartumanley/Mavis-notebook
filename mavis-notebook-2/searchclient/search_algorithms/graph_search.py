# coding: utf-8
#
# Copyright 2021 The Technical University of Denmark
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#    http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from __future__ import annotations
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import time
import memory
from typing import Union

import domains.hospital.actions as actions
import domains.hospital.state as state
import domains.hospital.goal_description as goal_description
import strategies.bfs as bfs

from domains.hospital.actions import AnyAction, MoveAction


# Note: This syntax below (<variable name>: <variable type>) is type hinting and is meant
# to make it easier for you to understand (now you know that `action_set` is a list of lists of
# actions!) but if it is confusing, you can just ignore it as it is only for documentation
def graph_search(
        initial_state:      state.HospitalState,
        action_set:         list[list[actions.AnyAction]],
        goal_description:   goal_description.HospitalGoalDescription,
        frontier:           bfs.FrontierBFS
    ) -> tuple[bool, list[list[actions.AnyAction]], int, float]:
    global start_time

    # Set start time
    start_time = time.time()
    iterations = 0 
    frontier.prepare(goal_description)

    # Clear the parent pointer and cost in order make sure that the initial state is a root node
    initial_state.parent = None
    initial_state.path_cost = 0

    # explored set to avoid cycles
    explored: set = set()
    # Add initial state to frontier
    frontier.add(initial_state)
    
    '''
    Implement the Graph-Search algorithm from R&N figure 3.7
    The algorithm should here return a (boolean, list[list[actions.AnyAction]], int, float) tuple, 
    where the boolean denotes whether the algorithm successfully found a plan, the list of lists 
    is the found plan, the integer is the number of states generated and the float is the amount of time
    elapsed. You can look at the print_search_status to find out how to get those quantities.

    You will need the following methods. 
    From the HospitalState class:
        state.extract_plan() - Returns the list of actions used to reach this state.
        state.get_applicable_actions(action_set) - Returns a list containing the actions applicable in the state.
        state.result(action) - Returns the new state reached by applying the action to the current state.
    From the HospitalGoalDescription class:
        goal_description.is_goal(node) - Returns true if the state is a goal state.
    From the FrontierBFS class:
        frontier.is_empty() - returns whether the frontier is emtpy
        frontier.pop() - pops the next node from the frontier, according to the search strategy
        frontier.add(state) - adds a state to the frontier, according to the search strategy
        frontier.contains(state) - checks whether the state is already present in the frontier

    Remember to look more into the different classes to see if there is any other 
    existing method that might help you out!    
    '''

    while True:
        # If the frontier is empty, return false, an empty plan, and the time it took
        if frontier.is_empty():
            elapsed_time = time.time() - start_time
            result: tuple =  False, [], iterations, elapsed_time 
            return  result
        else:
            # take the first node out of the pointer
            leaf: state.HospitalState = frontier.pop()
            iterations +=1 # increment node expansions

            # if node contains a goal state then return the corresponding solution
            if goal_description.is_goal(leaf):
                """The algorithm returns a (boolean, list[list[actions.AnyAction]], int, float) tuple, 
                where the boolean denotes whether the algorithm successfully found a plan, the list of lists 
                is the found plan, the integer is the number of states generated and the float is the amount of time
                elapsed."""

                # get num_generated and elapsed_time strings & convert to int and float
                num_generated_str, elapsed_time_str = print_search_status(leaf.extract_plan(), frontier)
                
                # would get conversion errors when trying to use the strings; i think it may be due to how the usage of ',' and '.' are swapped when it comes to US/EU numerical formats
                result: tuple = goal_description.is_goal(leaf), leaf.extract_plan(), float(num_generated_str), float(elapsed_time_str.replace(",", "."))
                return result
            
            # otherwise, add leaf to explored, expand the node and get all availiable actions that can be executed in current state
            explored.add(leaf)
            leaf_actions: list[AnyAction]  = leaf.get_applicable_actions(action_set)
                
            # for each availiable action at the leaf's current state
            for action in leaf_actions:
                # expand the node, add the resulting states to the frontier
                child_state =  leaf.result(action)

                # check if child is in frontier to prevent cycles
                if child_state not in explored and not frontier.contains(child_state):
                    frontier.add(child_state)


# A global variable used to keep track of the start time of the current search
start_time = 0


def print_search_status(expanded, frontier, print_search_meta_data=True):
    global start_time
    
    if len(expanded) == 0:
        start_time = time.time()
    memory_usage_bytes = memory.get_usage()
    
    # Replacing the generated comma thousands separators with dots is neither pretty nor locale aware but none of
    # Pythons four different formatting facilities seems to handle this correctly!
    num_expanded = f"{len(expanded):8,d}".replace(',', '.')
    num_frontier = f"{frontier.size():8,d}".replace(',', '.')
    num_generated = f"{len(expanded) + frontier.size():8,d}".replace(',', '.')
    elapsed_time = f"{time.time() - start_time:3.3f}".replace('.', ',')
    memory_usage_mb = f"{memory_usage_bytes / (1024*1024):3.2f}".replace('.', ',')
    status_text = f"#Expanded: {num_expanded}, #Frontier: {num_frontier}, #Generated: {num_generated}," \
                  f" Time: {elapsed_time} s, Memory: {memory_usage_mb} MB\n\n"
    
    if print_search_meta_data:
        print(status_text, file=sys.stderr)

    return num_generated, elapsed_time
