#pragma once
/*
 Sample code from https://www.redblobgames.com/pathfinding/a-star/
 Copyright 2014 Red Blob Games <redblobgames@gmail.com>

 Feel free to use this code in your own projects, including commercial projects
 License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>
*/

#include <iostream>
#include <set>
#include <map>
#include <array>
#include <vector>
#include <queue>
using namespace std;


struct GridLocation {
    int x, y;
};


struct SquareGrid {
    static array<GridLocation, 4> DIRS;

    int width, height;
    set<GridLocation> walls;
    int cost(GridLocation from_node, GridLocation to_node) const {
        return 1;
    }
    SquareGrid(int width_, int height_)
        : width(width_), height(height_) {}

    bool in_bounds(GridLocation id) const {
        return 0 <= id.x && id.x < width
            && 0 <= id.y && id.y < height;
    }

    bool passable(GridLocation id) const {
        return walls.find(id) == walls.end();
    }

    vector<GridLocation> neighbors(GridLocation id) const {
        vector<GridLocation> results;

        for (GridLocation dir : DIRS) {
            GridLocation next{ id.x + dir.x, id.y + dir.y };
            if (in_bounds(next) && passable(next)) {
                results.push_back(next);
            }
        }

        //if ((id.x + id.y) % 2 == 0) {
        //    // see "Ugly paths" section for an explanation:
        //    reverse(results.begin(), results.end());
        //}

        return results;
    }

    
};




//MAPPING ALGORITHM

struct Intersect {
    GridLocation loc;
    int cnt = 0; //count direction
    int neighbor[4] = { 0, 0, 0, 0 };  //West - North - East - South  0-available 1-available 2-gone
};

array<GridLocation, 4> SquareGrid::DIRS = {
    /* Left, Up, Right, Down */
    GridLocation{-1, 0}, GridLocation{0, 1},
    GridLocation{1, 0}, GridLocation{0, -1}
};

bool moved = false;

// Helpers for GridLocation

bool operator == (GridLocation a, GridLocation b) {
    return a.x == b.x && a.y == b.y;
}

bool operator != (GridLocation a, GridLocation b) {
    return !(a == b);
}

bool operator < (GridLocation a, GridLocation b) {
    return tie(a.x, a.y) < tie(b.x, b.y);
}

bool operator == (Intersect a, Intersect b) {
    return a.loc == b.loc;
}

int mod(int a, int b) {
    return (a % b < 0) ? (b + (a % b)) : (a % b);
}

SquareGrid convertToGrid(int map[8][8]) {
    SquareGrid grid(8, 8);
    for (int x = 0; x < 8; x++) {
        for (int y = 0; y < 8; y++) {
            if (map[x][y] == 0)     grid.walls.insert(GridLocation{ x, y });
        }
    }
    return grid;
}

void traverseMap(int map[8][8], GridLocation start, GridLocation loc, vector<GridLocation>* path, vector<Intersect> *intersect, int* dir) {
    //Add here: check movable direction, if it is intersection with sensor
    //Intersect its{ loc };
    int prev_dir = *dir;
    if (moved && loc.x == start.x && loc.y == start.y) {
        return;
    }
    bool left = false;      //check left
    bool forward = false;   //check up
    bool right = false;     //check right

    int l = mod(*dir - 1, 4);
    int r = mod(*dir + 1, 4);
    int f = *dir;
    int rv = mod((*dir - 2), 4);
    
    for (int i = 0; i < 4; i++) {
        GridLocation next{ loc.x + SquareGrid::DIRS[i].x, loc.y + SquareGrid::DIRS[i].y };
        if (next.x >= 0 && next.x < 8 && next.y >= 0 && next.y < 8 && map[next.x][next.y] == 1) {
            //its.cnt++;
            /*if (i != rv)    its.neighbor[i] = 1;*/
            if (i == l)    left = true;
            else if (i == f)     forward = true;
            else if (i == r)   right = true;
        }
    }

    if (left)
        *dir = l;  //go left if possible
    else if (!forward && right) 
        *dir = r;   //else go forward then right    
    else if ((!forward && !right))   
        *dir = rv;   //else reverse direction

    //Add here: movement
    GridLocation new_loc{ loc.x + SquareGrid::DIRS[*dir].x, loc.y + SquareGrid::DIRS[*dir].y };
    if (find(path->begin(), path->end(), new_loc) == path->end()) {
        path->push_back(new_loc);
    }
    moved = true;
    //Recursion
    traverseMap(map, start, new_loc, path, intersect, dir);
}


void mapGrid(int map[8][8], GridLocation start, GridLocation loc, vector<GridLocation>* path, vector<Intersect>* intersect, int* dir) {
    traverseMap(map, start, loc, path, intersect, dir);
    //remap(path);
    for (int x = 0; x < 8; x++) {
        for (int y = 0; y < 8; y++) {
            map[x][y] = 0;
        }
    }

    for (auto a : *path) {
        map[a.x][a.y] = 1;
    }

    for (int x = 0; x < 8; x++) {
        for (int y = 0; y < 8; y++) {
            if (map[y][x] == 0)     cout << "#";
            else cout << "@";
        }
        cout << '\n';
    }
}


//A* SEARCH ALGORITHM

// This outputs a grid. Pass in a distances map if you want to print
// the distances, or pass in a point_to map if you want to print
// arrows that point to the parent location, or pass in a path vector
// if you want to draw the path.
template<class Graph>
void draw_grid(const Graph& graph,
    map<GridLocation, int>* distances = nullptr,
    map<GridLocation, GridLocation>* point_to = nullptr,
    vector<GridLocation>* path = nullptr,
    GridLocation* start = nullptr,
    GridLocation* goal = nullptr) {
    const int field_width = 3;
    cout << string(field_width * graph.width, '_') << '\n';
    for (int y = 0; y != graph.height; ++y) {
        for (int x = 0; x != graph.width; ++x) {
            GridLocation id{ x, y };
            if (graph.walls.find(id) != graph.walls.end()) {
                cout << string(field_width, '#');
            }
            else if (start && id == *start) {
                cout << " A ";
            }
            else if (goal && id == *goal) {
                cout << " Z ";
            }
            else if (path != nullptr && find(path->begin(), path->end(), id) != path->end()) {
                cout << " @ ";
            }
            else if (point_to != nullptr && point_to->count(id)) {
                GridLocation next = (*point_to)[id];
                if (next.x == x + 1) { cout << " > "; }
                else if (next.x == x - 1) { cout << " < "; }
                else if (next.y == y + 1) { cout << " v "; }
                else if (next.y == y - 1) { cout << " ^ "; }
                else { cout << " * "; }
            }
            else if (distances != nullptr && distances->count(id)) {
                //cout << ' ' << left << setw(field_width - 1) << (*distances)[id];
            }
            else {
                cout << " . ";
            }
        }
        cout << '\n';
    }
    cout << string(field_width * graph.width, '~') << '\n';
}

void add_rect(SquareGrid grid, int x1, int y1, int x2, int y2) {
    for (int x = x1; x < x2; ++x) {
        for (int y = y1; y < y2; ++y) {
            grid.walls.insert(GridLocation{ x, y });
        }
    }
}

SquareGrid make_diagram1() {
    SquareGrid grid(30, 15);
    add_rect(grid, 3, 3, 5, 12);
    add_rect(grid, 13, 4, 15, 15);
    add_rect(grid, 21, 0, 23, 7);
    add_rect(grid, 23, 5, 26, 7);
    return grid;
}

struct GridWithWeights : SquareGrid {
    set<GridLocation> forests;
    GridWithWeights(int w, int h) : SquareGrid(w, h) {}
    int cost(GridLocation from_node, GridLocation to_node) const {
        return forests.find(to_node) != forests.end() ? 5 : 1;
    }
};

GridWithWeights make_diagram4() {
    GridWithWeights grid(10, 10);
    add_rect(grid, 1, 7, 4, 9);
    typedef GridLocation L;
    grid.forests = set<GridLocation>{
      L{3, 4}, L{3, 5}, L{4, 1}, L{4, 2},
      L{4, 3}, L{4, 4}, L{4, 5}, L{4, 6},
      L{4, 7}, L{4, 8}, L{5, 1}, L{5, 2},
      L{5, 3}, L{5, 4}, L{5, 5}, L{5, 6},
      L{5, 7}, L{5, 8}, L{6, 2}, L{6, 3},
      L{6, 4}, L{6, 5}, L{6, 6}, L{6, 7},
      L{7, 3}, L{7, 4}, L{7, 5}
    };
    return grid;
}

template<typename T, typename priority_t>
struct PriorityQueue {
    typedef pair<priority_t, T> PQElement;
    priority_queue<PQElement, vector<PQElement>,
        greater<PQElement>> elements;

    inline bool empty() const {
        return elements.empty();
    }

    inline void put(T item, priority_t priority) {
        elements.emplace(priority, item);
    }

    T get() {
        T best_item = elements.top().second;
        elements.pop();
        return best_item;
    }
};

template<typename Location>
vector<Location> reconstruct_path(Location start, Location goal, map<Location, Location> came_from) {
    vector<Location> path;
    Location current = goal;
    while (current != start) {
        path.push_back(current);
        current = came_from[current];
    }
    path.push_back(start); // optional
    reverse(path.begin(), path.end());
    return path;
}

inline int heuristic(GridLocation a, GridLocation b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

template<typename Location, typename Graph>
void a_star_search
(Graph graph,
    Location start,
    Location goal,
    map<Location, Location>& came_from,
    map<Location, int>& cost_so_far)
{
    PriorityQueue<Location, int> frontier;
    frontier.put(start, 0);

    came_from[start] = start;
    cost_so_far[start] = 0;

    while (!frontier.empty()) {
        Location current = frontier.get();

        if (current == goal) {
            break;
        }

        for (Location next : graph.neighbors(current)) {
            int new_cost = cost_so_far[current] + graph.cost(current, next);
            if (cost_so_far.find(next) == cost_so_far.end()
                || new_cost < cost_so_far[next]) {
                cost_so_far[next] = new_cost;
                int priority = new_cost + heuristic(next, goal);
                frontier.put(next, priority);
                came_from[next] = current;
            }
        }
    }
}