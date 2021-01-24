#include "Test.h"
//MAP ALGORITHM WITIH LEFT HAND RULE (LEFT WALL FOLLOWER)

int main() {

	int unmapped_map[8][8] =
	{{0, 0, 1, 1, 1, 1, 1, 1},
	 {0, 0, 0, 0, 1, 0, 0, 1},
	 {1, 1, 1, 1, 1, 1, 1, 1},
	 {0, 0, 0, 0, 1, 0, 0, 0},
	 {0, 0, 0, 1, 1, 1, 1, 1},
	 {0, 0, 0, 0, 1, 0, 0, 1},
	 {1, 1, 1, 1, 1, 1, 1, 1},
	 {0, 0, 0, 0, 0, 0, 0, 0},
	};
	GridLocation start1{ 2, 0 };
	int dir = 1;	// West - North - East - South
	vector<GridLocation> path1;
	vector<Intersect*> intersect;
	mapGrid(unmapped_map, start1, start1, &path1, &intersect, &dir);

	SquareGrid grid = convertToGrid(unmapped_map);
	GridLocation start{ 6, 0 }, goal{ 6, 6 };

	map<GridLocation, GridLocation> came_from;
	map<GridLocation, int> cost_so_far;
	a_star_search(grid, start, goal, came_from, cost_so_far);
	draw_grid(grid, nullptr, &came_from, nullptr, &start, &goal);
	cout << '\n';
	vector<GridLocation> path = reconstruct_path(start, goal, came_from);
	draw_grid(grid, nullptr, nullptr, &path, &start, &goal);
	cout << '\n';
	//draw_grid(grid, &cost_so_far, nullptr, nullptr, &start, &goal);

}