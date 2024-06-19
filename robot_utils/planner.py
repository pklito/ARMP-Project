import numpy as np
import time, sys
from RRT_Tree import RRTTree

class RRT_STAR(object):
    def __init__(self, max_step_size, max_itr, bb):
        self.max_step_size = max_step_size
        self.max_itr = max_itr
        self.bb = bb
        self.tree = RRTTree(bb)

    def compute_plan(self, plan, start_idx, goal_idx):
        curr_idx = goal_idx
        while curr_idx != start_idx:
            plan.append(self.tree.vertices[curr_idx])
            curr_idx = self.tree.edges[curr_idx]
        plan.append(self.tree.vertices[start_idx])
        plan.reverse()
        return plan

    def compute_cost(self, plan):
        '''
        Compute and return the plan cost, which is the sum of the distances between steps.
        @param plan A given plan for the robot.
        '''
        cost = 0
        for i in range(1, len(plan)):
            cost += self.bb.edge_cost(plan[i-1],plan[i])
        return cost

    def find_path(self, start_conf, goal_conf, filename):
        """Implement RRT-STAR"""

        i = 0
        self.tree.AddVertex(start_conf)
        plan = []
        goal_idx = None
        while i < self.max_itr:
            i += 1
            self.real_k = self.get_k_num(i)
            random_state = self.bb.sample(goal_conf)
            nearest_state_idx, nearest_state = self.tree.GetNearestVertex(random_state)
            new_state = self.extend(nearest_state, random_state)
            if not self.bb.is_in_collision(new_state) and self.bb.local_planner(nearest_state,new_state):
                if goal_idx == None:
                    print("iteration: " + str(i))
                else:
                    print("iteration: " + str(i) + " goal found")
                    if i % 100 == 0:
                        print(plan) # to enable early stopping
                new_state_idx = self.tree.AddVertex(new_state)
                if all(new_state == goal_conf):
                    goal_idx = new_state_idx
                self.tree.AddEdge(nearest_state_idx, new_state_idx)
                if len(self.tree.vertices) > self.real_k: # make sure the state has at least has k neighbors
                    k_nearest_idxs, k_nearest_states = self.tree.GetKNN(new_state, self.real_k)
                    for idx in k_nearest_idxs:
                        self.rewire(idx, new_state_idx)
                    for idx in k_nearest_idxs:
                        self.rewire(new_state_idx, idx)
            else:
                print("iteration: " + str(i) + " in Collision")
        if goal_idx != None:
            self.compute_plan(plan,0, goal_idx)
        return np.array(plan)

    def extend(self, x_near, x_random)-> np.array:
        '''
        Implement the Extend method
        @param x_near - Nearest Neighbor
        @param x_random - random sampled configuration
        return the extended configuration
        '''
        n = self.max_step_size # a changeable parameter for step-size
        dist = self.bb.edge_cost(x_near, x_random)
        if dist < n:
            return x_random
        normed_direction = (x_random - x_near) / dist # normed vector
        new_state = x_near + (n * normed_direction)
        return new_state

    def rewire_children(self, parent_idx, parent_cost):
        # Get the list of children vertices
        children_idxs = [idx for idx, parent in self.tree.edges.items() if parent == parent_idx]

        # Iterate through the children and rewire them if necessary
        for child_idx in children_idxs:
            child_vertex = self.tree.vertices[child_idx]

            # Recompute the cost of the child considering the new parent

            new_cost = parent_cost + self.bb.edge_cost(self.tree.vertices[parent_idx], child_vertex)

            # If the new cost is lower than the child's current cost, update the child's cost and rewire its children recursively

            _, child_cost = self.get_shortest_path(child_idx)
            if new_cost < child_cost:
                self.rewire_children(child_idx,new_cost)

    def rewire(self, x_potential_parent_id, x_child_id) -> None:
        '''
        Implement the rewire method
        @param x_potential_parent_id - candidte to become a parent
        @param x_child_id - the id of the child vertex
        return None
        '''

        if(x_child_id >= len(self.tree.vertices) or x_potential_parent_id >= len(self.tree.vertices)):
            return None

        # Get the child and potential parent vertices
        child_vertex = self.tree.vertices[x_child_id]
        potential_parent_vertex = self.tree.vertices[x_potential_parent_id]

        # Check if the edge between potential parent and child is valid
        if not self.bb.local_planner(potential_parent_vertex, child_vertex):
            return None

        # Compute the cost of the edge between potential parent and child
        edge_cost = self.bb.edge_cost(potential_parent_vertex, child_vertex)

        # Calculate the total cost if we rewire the child to the potential parent
        _, potential_parent_cost = self.get_shortest_path(x_potential_parent_id)
        total_cost = potential_parent_cost + edge_cost

        # Check if rewiring reduces the cost of the child
        _, child_cost = self.get_shortest_path(x_child_id)
        if not total_cost < child_cost:
            return None

        # Update the existing edge if it exists
        if x_child_id in self.tree.edges:
            # Update the cost of the child
            child_cost = total_cost
            # Update the parent of the child to the potential parent
            self.tree.edges[x_child_id] = x_potential_parent_id
            # Rewire children recursively if necessary
            self.rewire_children(x_child_id,potential_parent_cost)

    def get_shortest_path(self, dest):
        '''
        Returns the path and cost from some vertex to Tree's root
        @param dest - the id of some vertex
        return the shortest path and the cost
        '''
        # TODO
        path = self.compute_plan([],0, dest)
        cost = self.compute_cost(path)
        return path, cost

    def get_k_num(self, i):
        '''
        Determines the number of K nearest neighbors for each iteration
        '''
        if i < 300:
            k_num = 1
        elif 300 <= i < 600:
            k_num = 3
        elif 600 <= i < 1000:
            k_num=5
        elif 1000 <= i < 1500:
            k_num=6
        else:
            k_num = 7
        return k_num
