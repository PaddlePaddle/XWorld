"""
Copyright (c) 2017 Baidu Inc. All Rights Reserved.

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
"""

import random
import pprint


def flood_fill(seeds, obstacles, X, Y):
    """
    Starting from seeds, return a list of places that can be flooded
    """
    assert seeds
    visited = set(seeds)
    que = [s for s in seeds]  # deep copy
    while que:
        cur = que.pop(0)
        moves = [(-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0)]
        for m in moves:
            next = tuple([cur[i] + m[i] for i in range(len(cur))])
            if next[0] >= 0 and next[0] < X and next[1] >= 0 and next[1] < Y \
               and (not next in visited) and (not next in obstacles):
                visited.add(next)
                que.append(next)
    return list(visited - set(seeds))


## compute paths from the start to the end
def bfs(start, end, X, Y, obstacles):
    """
    return a shortest path from start to end
    """
    assert start != end
    que = [start]
    prev = {start: None}
    while que:
        cur = que.pop(0)
        if cur == end:
            break
        moves = [(-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0)]
        random.shuffle(moves)
        for m in moves:
            next = tuple([cur[i] + m[i] for i in range(len(cur))])
            if next[0] >= 0 and next[0] < X and next[1] >= 0 and next[1] < Y \
               and (not next in prev) and (not next in obstacles):
                prev[next] = cur
                que.append(next)
    ## if end is not reachable
    if cur != end:
        return
    ## backtrack
    track = []
    while cur is not None:
        track.append(cur)
        cur = prev[cur]
    assert len(track) >= 2
    return track[1:-1]


def spanning_tree_maze_generator(X, Y):
    """
    Generate a maze that has no loop
    """
    assert X == Y, "only support square maps"
    pad = False
    if X % 2 == 0:
        pad = True
        X, Y = X - 1, Y - 1

    visited = set([])
    maze = [[(' ' if x % 2 == 0 and y % 2 ==0 else '#') \
             for x in range(X)] for y in range(Y)]
    edges = set([])

    x, y = (X + 1) / 2, (Y + 1) / 2
    def dfs(cur):
        visited.add(cur)
        ## do not move moves outside
        moves = [(-1, 0), (1, 0), (0, 1), (0, -1)]
        random.shuffle(moves)
        for m in moves:
            next = (cur[0] + m[0], cur[1] + m[1])
            if not next in visited and next[0] >= 0 \
               and next[0] < x and next[1] >= 0 and next[1] < y:
                edges.add((cur, next))
                dfs(next)

    ## always start from (0, 0)
    dfs((0, 0))
    ## render the maze
    for e in edges:
        mid_x = e[0][0] + e[1][0]
        mid_y = e[0][1] + e[1][1]
        maze[mid_y][mid_x] = ' '

    if pad: # for even sizes, we pad the maze
        maze.append([' ' if i % 2 == 0 else '#' for i in range(X)])
        for i, m in enumerate(maze):
            m.append(' ' if i % 2 == 0 else '#')
    return maze


def print_env(X, Y, entities):
    """
    Print the current environment map for debugging purpose
    """
    m = [[' ' for x in range(X)] for y in range(Y)]
    for e in entities:
        if e.type == "goal":
            c = 'G'
        elif e.type == "block":
            c = 'B'
        else:
            assert e.type == "agent"
            c = 'A'
        x, y = e.loc[0], e.loc[1]
        x, y = int(x), int(y)
        assert y < len(m)
        assert x < len(m[Y-1-y]) # roboschool use a mirror coordinate system
        m[Y-1-y][x] = c
    print("Environment configure:")
    pprint.pprint(m)
