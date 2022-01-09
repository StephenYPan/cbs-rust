from sys import getsizeof
import time as timer
import heapq
import random

from collections import OrderedDict

from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost, build_mdd

import numpy as np


def detect_collision(path1, path2):
    for t in range(max(len(path1), len(path2))):
        cur_loc1 = get_location(path1, t)
        cur_loc2 = get_location(path2, t)
        prev_loc1 = get_location(path1, t - 1)
        prev_loc2 = get_location(path2, t - 1)
        if cur_loc1 == cur_loc2:  # Vertex collision
            return ([cur_loc1], t)
        if cur_loc1 == prev_loc2 and cur_loc2 == prev_loc1:  # Edge collision
            return ([cur_loc2, cur_loc1], t)
    return None


def detect_collisions(paths):
    result = []
    num_agents = len(paths)
    for i in range(num_agents - 1):
        for j in range(i + 1, num_agents):
            collision = detect_collision(paths[i], paths[j])
            if not collision:
                continue
            collision, timestep = collision
            result.append({
                'a1': i,
                'a2': j,
                'loc': collision,
                'timestep': timestep
            })
    return result


def standard_splitting(collision):
    result = []
    agent1 = collision['a1']
    agent2 = collision['a2']
    loc = collision['loc']
    timestep = collision['timestep']
    if len(loc) == 1:  # Vertex
        result.append({'agent': agent1, 'loc': loc, 'timestep': timestep,
                      'status': 'vertex', 'positive': False})
        result.append({'agent': agent2, 'loc': loc, 'timestep': timestep,
                      'status': 'vertex', 'positive': False})
    else:  # Edge
        edge1 = loc
        edge2 = [edge1[1], edge1[0]]
        result.append({'agent': agent1, 'loc': edge1,
                      'timestep': timestep, 'status': 'edge', 'positive': False})
        result.append({'agent': agent2, 'loc': edge2,
                      'timestep': timestep, 'status': 'edge', 'positive': False})
    return result


def disjoint_splitting(collision, mdds):
    """
    Weighted probabilities for choosing positive constraints between two agents. When one agent has
    no path in their MDD at timestep t, then all the weight falls onto the other agent. Otherwise,
    the probabilities are 50/50.
    """
    result = standard_splitting(collision)
    a1_weight = len([e for t, e in mdds[result[0]['agent']]
                    if t == result[0]['timestep']])
    a2_weight = len([e for t, e in mdds[result[1]['agent']]
                    if t == result[0]['timestep']])
    cum_weights = [1, 2]
    if not a1_weight or not a2_weight:  # Special case, if either are zero
        cum_weights = [a1_weight, a1_weight + a2_weight]
    population = [
        [result[0]['agent'], result[0]['loc']],
        [result[1]['agent'], result[1]['loc']]
    ]
    chosen_agent = random.choices(
        population=population, cum_weights=cum_weights)[0]
    for i, predicate in zip([0, 1], [True, False]):
        result[i]['agent'] = chosen_agent[0]
        result[i]['loc'] = chosen_agent[1]
        result[i]['positive'] = predicate
    return result


def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        cur = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == cur:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == cur \
                    or constraint['loc'] == [cur, prev]:
                rst.append(i)
    return rst


def gospers_hack(Set):
    # Gosper's hack
    # Cycle through all permutations of vertices to find a valid vertex cover with k bits
    c = Set & -Set
    r = Set + c
    return ((r ^ Set) >> 2) // c | r


def get_next_vertex_cover(graph, Set, V, E):
    # First get the next permutation of the set
    new_Set = gospers_hack(Set)
    Limit = 1 << V
    visited = np.empty(shape=(V, V))
    while new_Set < Limit:
        visited.fill(0)
        edge_count = 0
        k = 1
        i = 0
        while k < Limit:
            if new_Set & k:  # agent_i at position k exists in Set
                for j in range(V):
                    if not graph[i][j] or visited[i][j]:
                        continue
                    visited[i][j] = 1
                    visited[j][i] = 1
                    edge_count += 1
            k = k << 1
            i += 1
        if edge_count == E:
            return new_Set
        new_Set = gospers_hack(new_Set)
    return Set  # There is no other Set that can be a MVC


def is_vertex_cover(graph, V, k, E):
    # Author: GeeksforGeeks
    # Copyright: CC BY-SA
    # Source: https://www.geeksforgeeks.org/finding-minimum-vertex-cover-graph-using-binary-search/
    # Code was modified
    if E == 0:
        return (True, 0)
    if k == 0 and E > 0:
        return (False, 0)

    Set = (1 << k) - 1
    Limit = 1 << V
    visited = np.empty(shape=(V, V))
    while Set < Limit:
        visited.fill(0)
        edge_count = 0
        k = 1
        i = 0
        while k < Limit:
            if Set & k:  # agent_i at position k exists in Set
                for j in range(V):
                    if not graph[i][j] or visited[i][j]:
                        continue
                    visited[i][j] = 1
                    visited[j][i] = 1
                    edge_count += 1
            k = k << 1
            i += 1
        if edge_count == E:
            return (True, Set)
        Set = gospers_hack(Set)
    return (False, 0)


def min_vertex_cover(graph, V, E):
    """
    Returns the min vertex cover in bit permutation
    eg. 0010 = agent_1 is the vertex cover 
    """
    if E == 0:
        return (0, 0)
    left = 0
    # A better upperbound than |V|. |E| + 1 because of mid calculations
    right = min(E + 1, V)
    Set = 0
    while left < right:
        mid = left + right >> 1
        is_cover, cur_Set = is_vertex_cover(graph, V, mid, E)
        if is_cover:
            Set = cur_Set
            right = mid
        else:
            left = mid + 1
    return (left, Set)


def min_vertex_weight_min_vertex_cover(weight_adj_matrix, min_vertices, V):
    """
    Finds the minimum vertex weights for a given minimum vertex cover
    """
    cur_vertex_weights = np.zeros(V)
    # Iterate through vertices not in vertex cover to find the minimum viable vertex weight
    # for each edge weight_vu
    for v in min_vertices:
        for u in [i for i in range(V) if i not in min_vertices]:
            edge_weight = weight_adj_matrix[v][u]
            cur_vertex_weights[v] = max(cur_vertex_weights[v], edge_weight)
    # Adjust weights if v + u >= edge_weight condition does not satisfy
    for v in min_vertices:
        for u in min_vertices:
            edge_weight = weight_adj_matrix[v][u]
            v_weight = cur_vertex_weights[v]
            u_weight = cur_vertex_weights[u]
            if v_weight + u_weight >= edge_weight:
                continue
            cur_vertex_weights[v] += edge_weight - (v_weight + u_weight)
    return cur_vertex_weights


def find_extended_mdd_conflict(mdds, paths):
    """
    Return true if there exists a cardinal conflict with the extended mdd, otherwise false.
    """
    start = min(len(paths[0]), len(paths[1]))
    end = max(len(paths[0]), len(paths[1]))
    if start == end:
        return (False, None)
    if len(paths[0]) > len(paths[1]):
        mdds[0], mdds[1] = mdds[1], mdds[0]
        paths[0], paths[1] = paths[1], paths[0]
    vertex = paths[0][-1]
    mdd = [(t, e) for t, e in mdds[1] if t >= start]
    for i in range(start, end):
        mdd_vertex = set([e[1] for t, e in mdd if t == i])
        if len(mdd_vertex) == 1 and mdd_vertex == {vertex}:
            partial_collision = {'loc': [vertex], 'timestep': i}
            return (True, partial_collision)
    return (False, None)


def find_cardinal_conflict(mdds, paths):
    """
    Return true if there exists a cardinal conflict, otherwise false.
    """
    min_timestep = min(len(paths[0]), len(paths[1]))
    for i in range(1, min_timestep):
        agent1_edge = [(v, u) for t, (u, v) in mdds[0] if t == i]
        agent2_edge = [e for t, e in mdds[1] if t == i]
        if len(agent1_edge) == 1 and len(agent2_edge) == 1 and agent1_edge == agent2_edge:
            partial_collision = {'loc': list(agent1_edge.pop())[
                ::-1], 'timestep': i}
            return (True, partial_collision)
        agent1_vertex = set([e[0] for e in agent1_edge])
        agent2_vertex = set([e[1] for e in agent2_edge])
        if len(agent1_vertex) == 1 and len(agent2_vertex) == 1 and agent1_vertex == agent2_vertex:
            partial_collision = {'loc': [agent1_vertex.pop()], 'timestep': i}
            return (True, partial_collision)
    return find_extended_mdd_conflict(mdds, paths)


def find_dependency_conflict(mdds, paths):
    """
    Return true if there exists a dependency conflict, otherwise false.
    """
    min_timestep = min(len(paths[0]), len(paths[1]))
    joint_mdd = set()
    joint_mdd.add((0, (paths[0][0], paths[1][0])))
    for i in range(1, min_timestep):
        agent1_edge = [e for t, e in mdds[0] if t == i]
        agent2_edge = [e for t, e in mdds[1] if t == i]
        dependency_conflict = True
        for e1 in agent1_edge:
            for e2 in agent2_edge:
                if (i - 1, (e1[0], e2[0])) not in joint_mdd:
                    continue
                if e1[1] == e2[1]:  # Vertex collision
                    continue
                if e1[1] == e2[0] and e1[0] == e2[1]:  # Edge collision
                    continue
                dependency_conflict = False
                joint_mdd.add((i, (e1[1], e2[1])))
        if dependency_conflict:
            return True
    return find_extended_mdd_conflict(mdds, paths)[0]


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals,
                 h_cache=None, mdd_cache=None, low_lv_h_cache=None, partial_mdd_cache=None):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        self.disjoint = False
        self.stats = True
        self.cg_heuristics = False
        self.dg_heuristics = False
        self.wdg_heuristics = False

        self.agent_offset = None

        # Meta agent path finding
        self.meta = False

        # h-value statistics
        self.root_h_value = 0
        self.total_pop_h_value = 0
        self.total_push_h_value = 0
        self.num_of_pushes = 0

        # High level heuristics cache
        self.ewmvc_mvc_time = 0
        self.h_cache = h_cache if h_cache else OrderedDict()
        self.h_time = 0
        self.h_cache_hit_time = 0
        self.h_cache_miss_time = 0
        self.h_cache_max_size = 2**22  # 4 Mib, TODO: TUNE HYPERPARAMETER
        self.h_cache_hit = 0
        self.h_cache_miss = 0
        self.h_cache_evict_counter = 0

        # High level mdd cache
        self.mdd_cache = mdd_cache if mdd_cache else OrderedDict()
        self.mdd_time = 0
        self.mdd_pos_constraint_time = 0
        self.mdd_neg_constraint_time = 0
        self.mdd_clean_up_time = 0
        self.mdd_cache_hit_time = 0
        self.mdd_cache_miss_time = 0
        self.mdd_cache_max_size = 2**22  # 4 Mib, TODO: TUNE HYPERPARAMETER
        self.mdd_cache_hit = 0
        self.mdd_cache_miss = 0
        self.mdd_evict_counter = 0

        # Low-level heuristics cache
        self.low_lv_h_cache = low_lv_h_cache if low_lv_h_cache else OrderedDict()
        self.low_lv_h_time = 0
        self.low_lv_h_cache_hit_time = 0
        self.low_lv_h_cache_miss_time = 0
        self.low_lv_h_cache_max_size = 2**20  # 1 Mib, TODO: TUNE HYPERPARAMETER
        self.low_lv_h_cache_hit = 0
        self.low_lv_h_cache_miss = 0
        self.low_lv_h_cache_evict_counter = 0

        # Partial mdd cache
        self.partial_mdd_cache = partial_mdd_cache if partial_mdd_cache else OrderedDict()
        self.partial_mdd_time = 0
        self.partial_mdd_hit_time = 0
        self.partial_mdd_miss_time = 0
        self.partial_mdd_max_size = 2**20  # 1 Mib, TODO: TUNE HYPERPARAMETER
        self.partial_mdd_hit = 0
        self.partial_mdd_miss = 0
        self.partial_mdd_evict_counter = 0

        # compute heuristics for the low-level search
        self.goal_heuristics = []
        for goal in self.goals:
            self.goal_heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node, node_id):
        f_value = node['g_value'] + node['h_value']
        h_value = node['h_value']
        tie_break = len(node['collisions'])
        heapq.heappush(self.open_list, (f_value, h_value,
                       tie_break, node_id, node))
        # if self.stats:
        #     print('push -', 'sum:', f_value, ' h-value:', h_value, 'tie:', tie_break)
        self.num_of_pushes += 1

    def pop_node(self):
        # _, _, _, node_id, node = heapq.heappop(self.open_list)
        f, h, tie_break, node_id, node = heapq.heappop(self.open_list)
        if self.stats:
            print('pop -', 'f-value:', f, ' h-value:', h, 'tie:', tie_break)
        self.num_of_expanded += 1
        return (node, node_id)

    def prune_open_list(self, cost_cutoff):
        left = 0
        right = len(self.open_list) - 1
        while left < right:
            mid = left + right >> 1
            if self.open_list[mid][0] <= cost_cutoff:
                left = mid + 1
            else:
                right = mid
        self.open_list = self.open_list[0:right]

    def filter_mdd(self, mdd, path, neg_constraints):
        """
        Remove negative constraints from the MDD
        """
        neg_constraint_timer = timer.time()
        min_timestep = len(path)
        for timestep, loc in neg_constraints:
            if len(loc) == 1:
                loc = loc[0]
                # Remove vertices and the relating vertices in the next timestep
                edges_to_remove = [(t, e) for t, e in mdd if (t == timestep and e[1] == loc)
                                   or (t == timestep + 1 and e[0] == loc)]
                for t, e in edges_to_remove:
                    mdd.remove((t, e))
            else:
                loc = tuple(loc)
                if (timestep, loc) in mdd:  # MDD may not have the negative edge
                    mdd.remove((timestep, loc))
        self.mdd_neg_constraint_time += timer.time() - neg_constraint_timer
        # Clean up, remove non-connecting nodes
        clean_up_timer = timer.time()
        for i in range(min_timestep - 1, 1, -1):  # Remove backwards, nodes without children
            cur_vertex = set([e[0] for t, e in mdd if t == i])
            prev_t = i - 1
            prev_layer = [e for t, e in mdd if t ==
                          prev_t and e[1] not in cur_vertex]
            for e in prev_layer:
                mdd.remove((prev_t, e))
        for i in range(1, min_timestep - 1):  # Remove forward, nodes without parents
            cur_vertex = set([e[1] for t, e in mdd if t == i])
            next_t = i + 1
            next_layer = [e for t, e in mdd if t ==
                          next_t and e[0] not in cur_vertex]
            for e in next_layer:
                mdd.remove((next_t, e))
        for i, e in enumerate(zip(path, path[1:])):
            assert ((i + 1, e) in mdd) is True, f'filtered mdd does not contain path edge.\
                \nedge: {(i + 1, e)}\npath: {path}\nmdd: {[(t, e) for t, e in mdd if t == i + 1]}'
        self.mdd_clean_up_time += timer.time() - clean_up_timer
        return mdd

    def get_mdd_from_cache(self, agent, path, constraints):
        mdd_cache_timer = timer.time()
        mdd = None
        hash_value = hash(frozenset([(c['timestep'], tuple(c['loc']), c['positive'])
                                     for c in constraints]))
        hash_pair = (agent, len(path), hash_value)
        if hash_pair in self.mdd_cache:
            mdd = self.mdd_cache[hash_pair]
            self.mdd_cache.move_to_end(hash_pair)
            self.mdd_cache_hit += 1
            self.mdd_cache_hit_time += timer.time() - mdd_cache_timer
        else:
            # Try again for a cache hit with the old constraints iff the newly added constraint is
            # a negative constraint. This reduces the time spent filtering from scratch a brand new
            # MDD of depth n. Fetch the old MDD of depth n and adjust for the new negative
            # constraint.
            if constraints and not constraints[-1]['positive']:
                old_constraints = constraints[:len(constraints) - 1]
                old_hash_value = hash(frozenset([(c['timestep'], tuple(c['loc']), c['positive'])
                                                 for c in old_constraints]))
                old_hash_pair = (agent, len(path), old_hash_value)
                if old_hash_pair in self.mdd_cache:
                    neg_constraint = [
                        (constraints[-1]['timestep'], constraints[-1]['loc'])]
                    old_mdd = self.mdd_cache[old_hash_pair].copy()
                    mdd = self.filter_mdd(old_mdd, path, neg_constraint)
                    self.mdd_cache_hit += 1
            if not mdd:
                # Actual cache miss
                mdd = self.get_mdd(path, constraints)
                self.mdd_cache_miss += 1
            mdd_size = getsizeof(mdd)
            mdd_cache_size = getsizeof(self.mdd_cache)
            while mdd_cache_size + mdd_size > self.mdd_cache_max_size and len(self.mdd_cache) != 0:
                self.mdd_evict_counter += 1
                self.mdd_cache.popitem()
                mdd_cache_size = getsizeof(self.mdd_cache)
            self.mdd_cache[hash_pair] = mdd
            self.mdd_cache_miss_time += timer.time() - mdd_cache_timer
        return mdd

    def get_mdd(self, path, constraints):
        """
        Every positive constraint is the same as having intermediary goal nodes along the path
        towards the final goal. Therefore, you can calculate the MDD from (s to n), (n to m), 
        and so forth, up to (x to g) where each interval is an MDD that can be summed up to produce
        the MDD from (s to g). The edges and vertices from this MDD can then be removed given the
        negative constraints.
        """
        mdd = set()
        min_timestep = len(path)
        # Positive Constraints
        pos_constraint_timer = timer.time()
        pos_constraints = [(c['timestep'], c['loc']) for c in constraints
                           if c['positive'] == True and c['timestep'] < min_timestep]
        pos_vertex = set()
        for timestep, loc in pos_constraints:
            if len(loc) == 1:
                pos_vertex.add((timestep, loc[0]))
            else:
                loc = tuple(loc)
                pos_vertex.add((timestep - 1, loc[0]))
                pos_vertex.add((timestep, loc[1]))
        pos_vertex.add((0, path[0]))
        pos_vertex.add((min_timestep - 1, path[-1]))
        pos_vertex = sorted(pos_vertex)
        self.mdd_pos_constraint_time += timer.time() - pos_constraint_timer
        # Find MDD given intermediary goal nodes
        for start, goal in zip(pos_vertex, pos_vertex[1:]):
            # Cache Dijkstra results for start and goal locations
            low_level_h_timer = timer.time()
            h_values = [None, None]
            for i, location in enumerate([start[1], goal[1]]):
                ll_h_cache_timer = timer.time()
                if location in self.low_lv_h_cache:
                    h_values[i] = self.low_lv_h_cache[location]
                    self.low_lv_h_cache.move_to_end(location)
                    self.low_lv_h_cache_hit += 1
                    self.low_lv_h_cache_hit_time += timer.time() - ll_h_cache_timer
                else:
                    h_values[i] = compute_heuristics(self.my_map, location)
                    h_values_size = getsizeof(h_values[i])
                    h_cache_size = getsizeof(self.low_lv_h_cache)
                    while h_cache_size + h_values_size > self.low_lv_h_cache_max_size \
                            and len(self.low_lv_h_cache) != 0:
                        self.low_lv_h_cache_evict_counter += 1
                        self.low_lv_h_cache.popitem()
                        h_cache_size = getsizeof(self.low_lv_h_cache)
                    self.low_lv_h_cache[location] = h_values[i]
                    self.low_lv_h_cache_miss += 1
                    self.low_lv_h_cache_miss_time += timer.time() - ll_h_cache_timer
            self.low_lv_h_time += timer.time() - low_level_h_timer
            # Cache partial MDD
            partial_mdd_timer = timer.time()
            max_cost = goal[0] - start[0] + 1
            cost_offset = start[0]
            partial_mdd = None
            partial_mdd_key = (max_cost, cost_offset, start[1], goal[1])
            if partial_mdd_key in self.partial_mdd_cache:
                partial_mdd = self.partial_mdd_cache[partial_mdd_key]
                self.partial_mdd_cache.move_to_end(partial_mdd_key)
                self.partial_mdd_hit += 1
                self.partial_mdd_hit_time += timer.time() - partial_mdd_timer
            else:
                partial_mdd = build_mdd(
                    self.my_map, max_cost, cost_offset, h_values[0], h_values[1])
                partial_mdd_size = getsizeof(partial_mdd)
                partial_mdd_cache_size = getsizeof(self.partial_mdd_cache)
                while partial_mdd_cache_size + partial_mdd_size > self.partial_mdd_max_size \
                        and len(self.partial_mdd_cache) != 0:
                    self.partial_mdd_evict_counter += 1
                    self.partial_mdd_cache.popitem()
                    partial_mdd_cache_size = getsizeof(self.partial_mdd_cache)
                self.partial_mdd_cache[partial_mdd_key] = partial_mdd
                self.partial_mdd_miss += 1
                self.partial_mdd_miss_time += timer.time() - partial_mdd_timer
            mdd |= partial_mdd  # Set union
            self.partial_mdd_time += timer.time() - partial_mdd_timer
        # Negative Constraints
        neg_constraint_timer = timer.time()
        neg_constraints = [(c['timestep'], c['loc']) for c in constraints
                           if c['positive'] == False and c['timestep'] < min_timestep]
        if len(neg_constraints) == 0:  # Exit early, MDD was not modified
            for i, e in enumerate(zip(path, path[1:])):
                assert ((i + 1, e) in mdd) is True, f'mdd does not contain path edges.\
                    \nedge: {(i + 1, e)}\npath: {path}\
                    \nmdd: {[(t, e) for t, e in mdd if t == i + 1]}'
            self.mdd_neg_constraint_time += timer.time() - neg_constraint_timer
            return mdd
        return self.filter_mdd(mdd, path, neg_constraints)

    def cache_stats(self):
        stat_list = [
            [self.ewmvc_mvc_time, self.h_time, self.h_cache_hit_time, self.h_cache_miss_time,
                self.h_cache_hit, self.h_cache_miss, self.h_cache_evict_counter],
            [self.mdd_time, self.mdd_pos_constraint_time, self.mdd_neg_constraint_time,
                self.mdd_clean_up_time, self.mdd_cache_hit_time, self.mdd_cache_miss_time,
                self.mdd_cache_hit, self.mdd_cache_miss, self.mdd_evict_counter],
            [self.low_lv_h_time, self.low_lv_h_cache_hit_time, self.low_lv_h_cache_miss_time,
                self.low_lv_h_cache_hit, self.low_lv_h_cache_miss,
                self.low_lv_h_cache_evict_counter],
            [self.partial_mdd_time, self.partial_mdd_hit_time, self.partial_mdd_miss_time,
                self.partial_mdd_hit, self.partial_mdd_miss, self.partial_mdd_evict_counter]
        ]
        return stat_list

    def adjust_cache_stats(self, cache_stats):
        # High level heuristics cache
        self.ewmvc_mvc_time += cache_stats[0][0]
        self.h_time += cache_stats[0][1]
        self.h_cache_hit_time += cache_stats[0][2]
        self.h_cache_miss_time += cache_stats[0][3]
        self.h_cache_hit += cache_stats[0][4]
        self.h_cache_miss += cache_stats[0][5]
        self.h_cache_evict_counter += cache_stats[0][6]
        # High level mdd cache
        self.mdd_time += cache_stats[1][0]
        self.mdd_pos_constraint_time += cache_stats[1][1]
        self.mdd_neg_constraint_time += cache_stats[1][2]
        self.mdd_clean_up_time += cache_stats[1][3]
        self.mdd_cache_hit_time += cache_stats[1][4]
        self.mdd_cache_miss_time += cache_stats[1][5]
        self.mdd_cache_hit += cache_stats[1][6]
        self.mdd_cache_miss += cache_stats[1][7]
        self.mdd_evict_counter += cache_stats[1][8]
        # Low-level heuristics cache
        self.low_lv_h_time += cache_stats[2][0]
        self.low_lv_h_cache_hit_time += cache_stats[2][1]
        self.low_lv_h_cache_miss_time += cache_stats[2][2]
        self.low_lv_h_cache_hit += cache_stats[2][3]
        self.low_lv_h_cache_miss += cache_stats[2][4]
        self.low_lv_h_cache_evict_counter += cache_stats[2][5]
        # Partial mdd cache
        self.partial_mdd_time += cache_stats[3][0]
        self.partial_mdd_hit_time += cache_stats[3][1]
        self.partial_mdd_miss_time += cache_stats[3][2]
        self.partial_mdd_hit += cache_stats[3][3]
        self.partial_mdd_miss += cache_stats[3][4]
        self.partial_mdd_evict_counter += cache_stats[3][5]

    def bypass_collision(self, node):
        """
        Try to bypass the non-cardinal conflict and edit the path if possible.
        """
        start_num_collisions = len(node['collisions'])
        for collision in node['collisions']:
            constraints = disjoint_splitting(collision, node['mdds']) \
                if self.disjoint else standard_splitting(collision)
            for constraint in constraints:
                agent = constraint['agent']
                new_constraint = node['constraints'] + [constraint]
                new_path = a_star(self.my_map, self.starts[agent], self.goals[agent],
                                  self.goal_heuristics[agent], agent, new_constraint)
                if new_path is None:
                    continue
                # Edit the path without increasing the number of collisions. A bypass is successful
                # if the path length is the same and the number of collisions after the modification
                # is less than the previous number of collisions.
                if len(new_path) == len(node['paths'][agent]):
                    num_collisions = len(node['collisions'])
                    paths = [p for i, p in enumerate(
                        node['paths']) if i != agent] + [new_path]
                    new_num_collisions = len(detect_collisions(paths))
                    if new_num_collisions < num_collisions:
                        node['paths'][agent] = new_path
                        node['collisions'] = detect_collisions(node['paths'])
        new_num_collisions = len(detect_collisions(node['paths']))
        assert (start_num_collisions >= new_num_collisions) is True, \
            f'bypass logic error, collisions: {start_num_collisions} -> {new_num_collisions}'
        # if self.stats:
        #     print(f'bypass collisions: {start_num_collisions} -> {new_num_collisions}')
        node['g_value'] = get_sum_of_cost(node['paths'])
        return node['collisions'][0] if len(node['collisions']) else None

    def get_cardinal_collision(self, node):
        """
        Find and return cardinal conflicts if any within the list of collisions.
        """
        for c in node['collisions']:
            if not self.cg_heuristic(node['mdds'], node['paths'], [c]):
                continue
            a1 = c['a1']
            a2 = c['a2']
            mdds = [node['mdds'][a1], node['mdds'][a2]]
            paths = [node['paths'][a1], node['paths'][a2]]
            _, new_collision = find_cardinal_conflict(mdds, paths)
            new_collision['a1'] = a1
            new_collision['a2'] = a2
            return (True, new_collision)
        return (False, node['collisions'][0])

    def cg_heuristic(self, mdds, paths, collisions):
        """
        Constructs an adjacency matrix of cardinal conflicting agents
        and return the min vertex cover.
        """
        V = len(paths)
        E = 0
        adj_matrix = np.zeros(shape=(V, V))
        is_cardinal_conflict = False
        for c in collisions:
            h_start = timer.time()
            a1 = c['a1']
            a2 = c['a2']
            hash_value = hash(frozenset(mdds[a1])) ^ hash(frozenset(mdds[a2]))
            agent_hash_pair = (
                'cg', len(paths[a1]), len(paths[a2]), hash_value)
            if agent_hash_pair in self.h_cache:
                is_cardinal_conflict = self.h_cache[agent_hash_pair]
                self.h_cache.move_to_end(agent_hash_pair)
                self.h_cache_hit += 1
                self.h_cache_hit_time += timer.time() - h_start
            else:
                mdd_pair = [mdds[a1], mdds[a2]]
                path_pair = [paths[a1], paths[a2]]
                is_cardinal_conflict = find_cardinal_conflict(
                    mdd_pair, path_pair)[0]
                bool_size = getsizeof(is_cardinal_conflict)
                h_cache_size = getsizeof(self.h_cache)
                while h_cache_size + bool_size > self.h_cache_max_size and len(self.h_cache) != 0:
                    self.h_cache_evict_counter += 1
                    self.h_cache.popitem()
                    h_cache_size = getsizeof(self.h_cache)
                self.h_cache[agent_hash_pair] = is_cardinal_conflict
                self.h_cache_miss += 1
                self.h_cache_miss_time += timer.time() - h_start
            adj_matrix[a1][a2] = is_cardinal_conflict
            adj_matrix[a2][a1] = is_cardinal_conflict
            E += is_cardinal_conflict
        mvc_timer = timer.time()
        if E <= 1:  # Non-connected vertices or there has to be at least 1 vertex
            return E
        mvc_value, _ = min_vertex_cover(adj_matrix, V, E)
        self.ewmvc_mvc_time += timer.time() - mvc_timer
        return mvc_value

    def dg_heuristic(self, mdds, paths, collisions):
        """
        Constructs an adjacency matrix of dependency conflicts and return the minimum vertex cover.
        """
        V = len(paths)
        E = 0
        adj_matrix = np.zeros(shape=(V, V))
        is_dependency_conflict = False
        for c in collisions:
            h_start = timer.time()
            a1 = c['a1']
            a2 = c['a2']
            hash_value = hash(frozenset(mdds[a1])) ^ hash(frozenset(mdds[a2]))
            agent_hash_pair = (
                'dg', len(paths[a1]), len(paths[a2]), hash_value)
            if agent_hash_pair in self.h_cache:
                is_dependency_conflict = self.h_cache[agent_hash_pair]
                self.h_cache.move_to_end(agent_hash_pair)
                self.h_cache_hit += 1
                self.h_cache_hit_time += timer.time() - h_start
            else:
                mdd_pair = [mdds[a1], mdds[a2]]
                path_pair = [paths[a1], paths[a2]]
                is_dependency_conflict = find_dependency_conflict(
                    mdd_pair, path_pair)
                bool_size = getsizeof(is_dependency_conflict)
                h_cache_size = getsizeof(self.h_cache)
                while h_cache_size + bool_size > self.h_cache_max_size and len(self.h_cache) != 0:
                    self.h_cache_evict_counter += 1
                    self.h_cache.popitem()
                    h_cache_size = getsizeof(self.h_cache)
                self.h_cache[agent_hash_pair] = is_dependency_conflict
                self.h_cache_miss += 1
                self.h_cache_miss_time += timer.time() - h_start
            adj_matrix[a1][a2] = is_dependency_conflict
            adj_matrix[a2][a1] = is_dependency_conflict
            E += is_dependency_conflict
        mvc_timer = timer.time()
        if E <= 1:  # Non-connected vertices or there has to be at least 1 vertex
            return E
        mvc_value, _ = min_vertex_cover(adj_matrix, V, E)
        self.ewmvc_mvc_time += timer.time() - mvc_timer
        return mvc_value

    def wdg_heuristic(self, mdds, paths, collisions, constraints):
        """
        Construct a weighted dependency graph and return the edge weight minimum vertex cover
        """
        V = len(paths)
        E = 0
        adj_matrix = np.zeros(shape=(V, V))
        vertex_weights = [0] * V
        for collision in collisions:
            if not self.dg_heuristic(mdds, paths, [collision]):
                continue
            # Find the edge weight for the dependency conflict
            h_start = timer.time()
            a1 = collision['a1']
            a2 = collision['a2']
            edge_weight = 0
            hash_value = hash(frozenset(mdds[a1])) ^ hash(frozenset(mdds[a2]))
            agent_hash_pair = ('wdg', len(
                paths[a1]), len(paths[a2]), hash_value)
            if agent_hash_pair in self.h_cache:
                edge_weight = self.h_cache[agent_hash_pair]
                self.h_cache.move_to_end(agent_hash_pair)
                self.h_cache_hit += 1
                self.h_cache_hit_time += timer.time() - h_start
            else:
                substarts = [self.starts[a1], self.starts[a2]]
                subgoals = [self.goals[a1], self.goals[a2]]
                subconstraints = [c for c in constraints if (
                    c['agent'] == a1 or c['agent'] == a2)]
                # Modify the slice of the original list
                for c in subconstraints:
                    c['agent'] = int(c['agent'] == a2)
                # Offset is used for cache hits. a2 is guaranteed to be bigger than 0 because
                # of the ordering for detect_collision
                agent_offset = [self.agent_offset[a1] +
                                a1, self.agent_offset[a2] + a2 - 1]
                # Run a relaxed cbs problem
                cbs_start = timer.time()
                cbs = CBSSolver(my_map=self.my_map, starts=substarts, goals=subgoals,
                                h_cache=self.h_cache, mdd_cache=self.mdd_cache,
                                low_lv_h_cache=self.low_lv_h_cache, partial_mdd_cache=self.partial_mdd_cache)
                new_paths, cache_stats = cbs.find_solution(disjoint=self.disjoint, stats=False,
                                                           cg_heuristics=self.cg_heuristics, dg_heuristics=self.dg_heuristics,
                                                           constraints=subconstraints, agent_offset=agent_offset)
                cbs_end = timer.time() - cbs_start
                # Undo the modification to the slice of the original list
                agent_dict = {i + a: i for i,
                              a in enumerate(self.agent_offset)}
                for c in subconstraints:
                    c['agent'] = agent_dict[agent_offset[c['agent']] + c['agent']]
                # The time spent performing cbs search should be categorized as miss time
                cbs_cpu_time = cbs_end - cache_stats[0][1] - cache_stats[1][0]
                # Only remove the redistributed times
                self.h_time -= cbs_end - cbs_cpu_time
                self.adjust_cache_stats(cache_stats)
                if new_paths:
                    # Get the maximum edge weight
                    a1_path_diff = len(new_paths[0]) - len(paths[a1])
                    a2_path_diff = len(new_paths[1]) - len(paths[a2])
                    edge_weight = max(a1_path_diff, a2_path_diff, 1)
                else:
                    # The collision may not produce a solution. Defaults to 1 like dg_heuristic
                    edge_weight = 1
                int_size = getsizeof(edge_weight)
                h_cache_size = getsizeof(self.h_cache)
                while h_cache_size + int_size > self.h_cache_max_size and len(self.h_cache) != 0:
                    self.h_cache_evict_counter += 1
                    self.h_cache.popitem()
                    h_cache_size = getsizeof(self.h_cache)
                self.h_cache[agent_hash_pair] = edge_weight
                self.h_cache_miss += 1
                self.h_cache_miss_time += timer.time() - h_start - cbs_end + cbs_cpu_time
            adj_matrix[a1][a2] = edge_weight
            adj_matrix[a2][a1] = edge_weight
            vertex_weights[a1] = max(vertex_weights[a1], edge_weight)
            vertex_weights[a2] = max(vertex_weights[a2], edge_weight)
            E += 1
        mvc_timer = timer.time()
        if E <= 1:
            return sum(vertex_weights) >> 1
        _, Set = min_vertex_cover(adj_matrix, V, E)
        # Find the first min vertex weight with the first found MVC
        # If |V| / 2 >> |next_Set|, then iterating through next_Set is faster. Since MVCs are
        # at worst case equal to |V| / 2, iterating through next_Set is faster on avg.
        # mvc_agents = [k for k in range(V) if Set & (1 << k)]
        bit_len = Set.bit_length() - 1
        mvc_agents = [bit_len - i for i,
                      k in enumerate(list(format(Set, 'b'))) if int(k)]
        new_vertex_weights = min_vertex_weight_min_vertex_cover(
            adj_matrix, mvc_agents, V)
        vertex_weight_diff = sum(vertex_weights) - sum(new_vertex_weights)
        min_vertex_weight_value = sum(vertex_weights)
        if vertex_weight_diff > 0:
            vertex_weights = new_vertex_weights
            min_vertex_weight_value -= vertex_weight_diff
        # Find the next mvc and look for a new min vertex weight
        cur_Set = Set
        next_Set = get_next_vertex_cover(adj_matrix, cur_Set, V, E)
        while next_Set != cur_Set:
            bit_len = next_Set.bit_length() - 1
            mvc_agents = [bit_len - i for i,
                          k in enumerate(list(format(next_Set, 'b'))) if int(k)]
            new_vertex_weights = min_vertex_weight_min_vertex_cover(
                adj_matrix, mvc_agents, V)
            vertex_weight_diff = sum(vertex_weights) - sum(new_vertex_weights)
            if vertex_weight_diff > 0:
                Set = next_Set  # Save lowest ewmvc bit set
                vertex_weights = new_vertex_weights
                min_vertex_weight_value -= vertex_weight_diff
            cur_Set = next_Set
            next_Set = get_next_vertex_cover(adj_matrix, cur_Set, V, E)
        self.ewmvc_mvc_time += timer.time() - mvc_timer
        return min_vertex_weight_value

    def get_heuristics(self, i, node):
        result = 0
        if i == 0:
            result = self.cg_heuristic(
                node['mdds'], node['paths'], node['collisions'])
        if i == 1:
            result = self.dg_heuristic(
                node['mdds'], node['paths'], node['collisions'])
        if i == 2:
            result = self.wdg_heuristic(node['mdds'], node['paths'], node['collisions'],
                                        node['constraints'])
        return int(result)

    def get_adj_matrix(self, node):
        V = self.num_of_agents
        adj_matrix = np.zeros(shape=(V, V), dtype=bool)
        for c in node['collisions']:
            h_time = timer.time()
            if not self.dg_heuristic(node['mdds'], node['paths'], [c]):
                self.h_time += timer.time() - h_time
                continue
            self.h_time += timer.time() - h_time
            a1 = c['a1']
            a2 = c['a2']
            adj_matrix[a1][a2] = 1
            adj_matrix[a2][a1] = 1
        return adj_matrix

    def get_adj_list(self, adj_matrix):
        return {i: np.nonzero(row)[0].tolist() for i, row in enumerate(adj_matrix) if sum(row) > 0}

    def get_subgraphs(self, adj_list):
        V = self.num_of_agents
        visited = np.zeros(V)
        open_list = set()
        graph = []
        for v in adj_list:
            if visited[v]:
                continue
            visited[v] = 1
            subgroup = {v}
            open_list |= set(adj_list[v])
            while open_list:
                next_v = open_list.pop()
                if visited[next_v]:
                    continue
                subgroup.add(next_v)
                visited[next_v] = 1
                if next_v in adj_list:
                    open_list |= set(adj_list[next_v])
            graph.append(subgroup)
        return graph

    def meta_agent_path_finding(self, node):
        # Get the adj_matrix of the new collisions
        # Find a larger dependency group with the parent adj_matrix and the new_adj_matrix
        # By adding the two matrices together without doing complicated athematic.
        parent_graph = self.get_subgraphs(
            self.get_adj_list(node['adj_matrix']))
        adj_matrix = self.get_adj_matrix(node)
        node['adj_matrix'] += adj_matrix
        adj_list = self.get_adj_list(node['adj_matrix'])
        meta_start = timer.time()
        cur_graph = self.get_subgraphs(self.get_adj_list(adj_matrix))
        graph = self.get_subgraphs(adj_list)
        # Solve the one dependency outside
        if len(graph) <= 1:
            return True
        cur_cost = get_sum_of_cost(node['paths'])
        for subgraph in graph:
            assert (
                len(subgraph) > 1) is True, f'subgraph has length: {len(subgraph)}, {subgraph}'
            substarts = [self.starts[a] for a in subgraph]
            subgoals = [self.goals[a] for a in subgraph]
            subconstraints = [c for c in node['constraints']
                              if c['agent'] in subgraph]
            subgraph_dict = {a: i for i, a in enumerate(subgraph)}
            # Modify constraints
            for c in subconstraints:
                c['agent'] = subgraph_dict[c['agent']]
            # Offset for cache hit
            agent_offset = [a - i for i, a in enumerate(subgraph)]
            cbs_start = timer.time()
            cbs = CBSSolver(my_map=self.my_map, starts=substarts, goals=subgoals,
                            h_cache=self.h_cache, mdd_cache=self.mdd_cache,
                            low_lv_h_cache=self.low_lv_h_cache, partial_mdd_cache=self.partial_mdd_cache)
            new_paths, cache_stats = cbs.find_solution(disjoint=self.disjoint, stats=False,
                                                       cg_heuristics=self.cg_heuristics, dg_heuristics=self.dg_heuristics,
                                                       wdg_heuristics=self.wdg_heuristics, constraints=subconstraints,
                                                       agent_offset=agent_offset)
            cbs_end = timer.time() - cbs_start
            self.adjust_cache_stats(cache_stats)
            # Revert modification to constraints
            subgraph_dict = {i: a for i, a in enumerate(subgraph)}
            for c in subconstraints:
                c['agent'] = subgraph_dict[c['agent']]
            # TODO: Should we immediately return no path and skip this whole branch?
            if not new_paths:  # No solution
                print(
                    f'No solution in cost: {cur_cost}, meta agent: {subgraph} in {graph}')
                return False
            for i, agent in enumerate(subgraph):
                node['paths'][agent] = new_paths[i]
            subgraph_conflicts = detect_collisions(node['paths'])
            for conflict in subgraph_conflicts:
                a1 = conflict['a1']
                a2 = conflict['a2']
                assert (
                    a1 in subgraph and a2 in subgraph) is False, f'{subgraph}, {conflict}'
        new_cost = get_sum_of_cost(node['paths'])
        node['collisions'] = detect_collisions(node['paths'])
        node['g_value'] = new_cost
        num_col = len(node['collisions'])
        print(f'Cost: {cur_cost} -> {new_cost}, # of Collisions: {num_col}')
        print(f'DG: {parent_graph} + {cur_graph} -> {graph}\n')
        meta_end = timer.time() - meta_start
        return True

    def find_solution(self, disjoint=False, cg_heuristics=False, dg_heuristics=False,
                      wdg_heuristics=False, meta=False, stats=True, constraints=None, agent_offset=None):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint        - use disjoint splitting or not
        cg_heuristics   - use conflict graph heuristics
        dg_heuristics   - use dependency graph heuristics
        wdg_heuristics  - use weighted dependency graph heuristics
        agent_offset    - cache utilization when running cbs on pairs of agent in wdg_heuristic
        """
        self.disjoint = disjoint
        self.stats = stats
        self.cg_heuristics = cg_heuristics
        self.dg_heuristics = dg_heuristics
        self.wdg_heuristics = wdg_heuristics
        self.meta = meta

        self.start_time = timer.time()

        heuristics = [self.cg_heuristics,
                      self.dg_heuristics, self.wdg_heuristics]
        lazy_a_star = [i for i, h in enumerate(heuristics) if h]
        # The case when no heuristics are enabled
        lazy_a_star = lazy_a_star if lazy_a_star else [-1]

        root = {
            'g_value': 0,
            'heuristic': lazy_a_star[-1],
            'h_value': 0,
            'constraints': constraints if constraints else [],
            'paths': [],
            'collisions': [],
            'mdds': [None] * self.num_of_agents,
            'adj_matrix': None
        }
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.goal_heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        if not agent_offset:
            self.agent_offset = np.zeros(self.num_of_agents)
        else:
            self.agent_offset = agent_offset

        root['collisions'] = detect_collisions(root['paths'])

        # Fetch MDD from cache
        mdd_start = timer.time()
        for i in range(self.num_of_agents):
            constraints = [c for c in root['constraints'] if c['agent'] == i]
            root['mdds'][i] = self.get_mdd_from_cache(i + self.agent_offset[i], root['paths'][i],
                                                      constraints)
        self.mdd_time += timer.time() - mdd_start

        heuristics_start = timer.time()
        root['h_value'] = self.get_heuristics(lazy_a_star[-1], root)
        self.h_time += timer.time() - heuristics_start
        self.root_h_value = root['h_value']
        root['g_value'] = get_sum_of_cost(root['paths'])

        if self.meta:
            root['adj_matrix'] = self.get_adj_matrix(root)

        self.push_node(root, self.num_of_generated)
        self.num_of_generated += 1

        while self.open_list:
            cur_node, cur_node_id = self.pop_node()
            self.total_pop_h_value += cur_node['h_value']
            if not cur_node['collisions']:  # Goal reached
                self.CPU_time = timer.time() - self.start_time
                if self.stats:
                    self.print_results(cur_node)
                return (cur_node['paths'], self.cache_stats())
            # If lazy A* is enabled, then the node is not expanded and pushed back into the open
            # list. We only increase the number expanded if the node is not pushed back into the
            # open list.
            if len(lazy_a_star) > 1 and cur_node['heuristic'] != lazy_a_star[-1]:
                heuristics_start = timer.time()
                cur_node['heuristic'] = lazy_a_star[-1]
                h_value = self.get_heuristics(lazy_a_star[-1], cur_node)
                cur_node['h_value'] = max(cur_node['h_value'], h_value)
                self.h_time += timer.time() - heuristics_start
                self.push_node(cur_node, cur_node_id)
                self.total_push_h_value += h_value
                self.num_of_expanded -= 1
                continue

            # Meta Agent
            # After meta agent, run cheap heuristic on the new paths and push them to the open list.
            # Meta agent is run only for the nodes expanded with expensive heuristics. Once meta
            # agent finds a path for each meta group, the solution is paired with the cheap
            # heuristic. If the solution from meta agent and the cheap heuristic is popped, an
            # expensive heuristic is then calculated and the node is push back to the open list.
            if self.meta:
                # Skip, no solution in a meta agent
                if not self.meta_agent_path_finding(cur_node):
                    continue
                if not cur_node['collisions']:  # Goal reached
                    cur_node['h_value'] = 0
                    self.push_node(cur_node, cur_node_id)
                    self.num_of_expanded -= 1
                    continue
                # Get new set of MDDs for heuristics calculation of meta agent paths
                mdd_start = timer.time()
                for i in range(self.num_of_agents):
                    constraints = [
                        c for c in cur_node['constraints'] if c['agent'] == i]
                    cur_node['mdds'][i] = self.get_mdd_from_cache(i + self.agent_offset[i],
                                                                  cur_node['paths'][i], constraints)
                self.mdd_time += timer.time() - mdd_start

            # Find and split on cardinal conflicts if any
            is_cardinal, collision = self.get_cardinal_collision(cur_node)
            if not is_cardinal:
                collision = self.bypass_collision(cur_node)
                if not cur_node['collisions']:
                    cur_node['h_value'] = 0
                    self.push_node(cur_node, cur_node_id)
                    self.num_of_expanded -= 1
                    continue
            constraints = disjoint_splitting(collision, cur_node['mdds']) \
                if disjoint else standard_splitting(collision)
            for constraint in constraints:
                new_node = {
                    'g_value': 0,
                    'heuristic': lazy_a_star[0],
                    'h_value': 0,
                    'constraints': [],
                    'paths': [],
                    'collisions': [],
                    'mdds': [None] * self.num_of_agents,
                    'adj_matrix': cur_node['adj_matrix']
                }

                # new_node['constraints'] = cur_node['constraints'] + [constraint]
                new_node['constraints'] = cur_node['constraints'] \
                    + [c for c in [constraint] if c not in cur_node['constraints']]
                agent = constraint['agent']
                path = a_star(self.my_map, self.starts[agent], self.goals[agent],
                              self.goal_heuristics[agent], agent, new_node['constraints'])
                if not path:
                    continue
                new_node['paths'] = cur_node['paths'].copy()  # Shallow copy
                new_node['paths'][agent] = path  # Edit shallow copy

                # Disjoint. Don't add the child node if there exists another agent with no path
                skip = False
                if constraint['positive']:
                    agent_ids = paths_violate_constraint(
                        constraint, new_node['paths'])
                    for i in agent_ids:
                        if constraint['status'] == 'vertex':
                            new_node['constraints'].append({
                                'agent': i,
                                'loc': constraint['loc'],
                                'timestep': constraint['timestep'],
                                'status': 'vertex',
                                'positive': False
                            })
                        else:
                            # Add two negative vertex constraints to the agent, since the positive
                            # edge constraint (u, v) at timestep t forces the agent to be at
                            # vertex u at timestep t-1 and vertex v at timestep t
                            for j in range(2):
                                new_node['constraints'].append({
                                    'agent': i,
                                    'loc': [constraint['loc'][j]],
                                    'timestep': constraint['timestep'] + j - 1,
                                    'status': 'vertex',
                                    'positive': False
                                })
                            new_node['constraints'].append({
                                'agent': i,
                                'loc': [constraint['loc'][1], constraint['loc'][0]],
                                'timestep': constraint['timestep'],
                                'status': 'edge',
                                'positive': False
                            })
                        path_i = a_star(
                            self.my_map, self.starts[i], self.goals[i], self.goal_heuristics[i], i, new_node['constraints'])
                        if not path_i:
                            skip = True
                            break
                        new_node['paths'][i] = path_i
                    if skip:
                        continue

                new_node['collisions'] = detect_collisions(new_node['paths'])

                # Fetch MDD from cache
                mdd_start = timer.time()
                for i in range(self.num_of_agents):
                    constraints = [
                        c for c in new_node['constraints'] if c['agent'] == i]
                    new_node['mdds'][i] = self.get_mdd_from_cache(i + self.agent_offset[i], new_node['paths'][i],
                                                                  constraints)
                self.mdd_time += timer.time() - mdd_start

                heuristics_start = timer.time()
                h_value = self.get_heuristics(lazy_a_star[0], new_node)
                new_node['h_value'] = h_value
                self.h_time += timer.time() - heuristics_start

                new_node['g_value'] = get_sum_of_cost(new_node['paths'])

                # for i in range(self.num_of_agents):
                #     print(f'agent-{i} path:', new_node['paths'][i])
                self.push_node(new_node, self.num_of_generated)
                self.total_push_h_value += h_value
                self.num_of_generated += 1

        return (None, self.cache_stats())  # Failed to find solutions

    def get_results(self):
        result = []
        return result

    def print_results(self, node):
        # print("\n Found a solution! \n")
        print()
        overhead = self.mdd_time + self.h_time
        search_time = self.CPU_time - overhead
        paths = node['paths']
        print(f'CPU time (s):       {self.CPU_time:.2f}')
        print(
            f'Search time:        {search_time:.2f} ({search_time / self.CPU_time * 100:05.2f}%)')
        print(
            f'Overhead time:      {overhead:.2f} ({overhead / self.CPU_time * 100:05.2f}%)')
        print(f'Overhead ratio:     {overhead / search_time:.2f}:1')
        print(f'Root h-value:       {self.root_h_value}')
        print(
            f'Avg pop h-value:    {self.total_pop_h_value / self.num_of_expanded:.2f}')
        print(
            f'Avg push h-value:   {self.total_push_h_value / self.num_of_pushes:.2f}')
        print(f'Heuristics cache:   {getsizeof(self.h_cache)} (bytes)')
        print(f'Heuristics time:    {self.h_time:.2f}')
        print(
            f' - EWMVC/MVC time:  {self.ewmvc_mvc_time:.2f} ({self.ewmvc_mvc_time / self.h_time * 100:05.2f}%)')
        print(
            f' - Hit time:        {self.h_cache_hit_time:.2f} ({self.h_cache_hit_time / self.h_time * 100:05.2f}%)')
        print(
            f' - Miss time:       {self.h_cache_miss_time:.2f} ({self.h_cache_miss_time / self.h_time * 100:05.2f}%)')
        print(f' - Hit/miss ratio:  {self.h_cache_hit}:{self.h_cache_miss}')
        print(f' - Evicted #:       {self.h_cache_evict_counter}')
        print(f'MDD cache:          {getsizeof(self.mdd_cache)} (bytes)')
        print(f'MDD time:           {self.mdd_time:.2f}')
        print(
            f' - Hit time:        {self.mdd_cache_hit_time:.2f} ({self.mdd_cache_hit_time / self.mdd_time * 100:05.2f}%)')
        print(
            f' - Miss time:       {self.mdd_cache_miss_time:.2f} ({self.mdd_cache_miss_time / self.mdd_time * 100:05.2f}%)')
        print(f'    - Positive time:     {self.mdd_pos_constraint_time:.2f}')
        print(
            f'    - Dijkstra cache:    {getsizeof(self.low_lv_h_cache)} (bytes)')
        print(f'    - Dijkstra time:     {self.low_lv_h_time:.2f}')
        print(f'       - Hit time:       {self.low_lv_h_cache_hit_time:.2f}')
        print(f'       - Miss time:      {self.low_lv_h_cache_miss_time:.2f}')
        print(
            f'       - Hit/miss ratio: {self.low_lv_h_cache_hit}:{self.low_lv_h_cache_miss}')
        print(f'       - Evicted #:      {self.low_lv_h_cache_evict_counter}')
        print(
            f'    - Partial MDD cache: {getsizeof(self.partial_mdd_cache)} (bytes)')
        print(f'    - Partial MDD time:  {self.partial_mdd_time:.2f}')
        print(f'       - Hit time:       {self.partial_mdd_hit_time:.2f}')
        print(f'       - Miss time:      {self.partial_mdd_miss_time:.2f}')
        print(
            f'       - Hit/miss ratio: {self.partial_mdd_hit}:{self.partial_mdd_miss}')
        print(f'       - Evicted #:      {self.partial_mdd_evict_counter}')
        print(f'    - Negative time:     {self.mdd_neg_constraint_time:.2f}')
        print(f'    - Clean up:          {self.mdd_clean_up_time:.2f}')
        print(
            f' - Hit/miss ratio:  {self.mdd_cache_hit}:{self.mdd_cache_miss}')
        print(f' - Evicted #:       {self.mdd_evict_counter}')
        print(f'Sum of costs:       {get_sum_of_cost(paths)}')
        print(f'Expanded nodes:     {self.num_of_expanded}')
        print(f'Generated nodes:    {self.num_of_generated}')
