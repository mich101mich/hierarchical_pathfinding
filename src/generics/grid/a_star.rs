use super::super::{ordered_insert, Cost, Path};
use std::collections::HashMap;
use std::hash::Hash;

/// Searches a Graph using the [A* Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm).
///
/// The Generic type Parameter `Id` is supposed to uniquely identify a Node in the Graph.
/// This may be a Number, String, a Grid position, ... as long as it can be compared, hashed and copied.
/// Note that it is advised to choose a short representation for the Id, since it will be copied several times.
///
/// ## Examples
/// Basic usage:
/// ```
/// # use hierarchical_pathfinding::generics::grid::a_star_search;
/// # use hierarchical_pathfinding::{prelude::*, Point};
/// // create and initialize Grid
/// // 0 = empty, 1 = swamp, 2 = wall
/// let mut grid = [
///     [0, 2, 0, 0, 0],
///     [0, 2, 2, 2, 2],
///     [0, 1, 0, 0, 0],
///     [0, 1, 0, 2, 0],
///     [0, 0, 0, 2, 0],
/// ];
/// let (width, height) = (grid.len(), grid[0].len());
///
/// let neighborhood = ManhattanNeighborhood::new(width, height);
///
/// const COST_MAP: [isize; 3] = [1, 10, -1];
///
/// fn cost_fn<'a>(grid: &'a [[usize; 5]; 5]) -> impl 'a + FnMut(Point) -> isize {
///     move |(x, y)| COST_MAP[grid[y][x]]
/// }
///
/// let start = (0, 0);
/// let goal = (4, 4);
///
/// let path = a_star_search(
///     |point| neighborhood.get_all_neighbors(point),
///     cost_fn(&grid),
///     start,
///     goal,
///     |point| neighborhood.heuristic(point, goal),
/// );
///
/// assert!(path.is_some());
/// let path = path.unwrap();
///
/// assert_eq!(path.cost(), 12);
/// ```
///
/// If the Goal cannot be reached, None is returned:
/// ```
/// # use hierarchical_pathfinding::generics::grid::a_star_search;
/// # use hierarchical_pathfinding::{prelude::*, Point};
/// # // create and initialize Grid
/// # // 0 = empty, 1 = swamp, 2 = wall
/// # let mut grid = [
/// #     [0, 2, 0, 0, 0],
/// #     [0, 2, 2, 2, 2],
/// #     [0, 1, 0, 0, 0],
/// #     [0, 1, 0, 2, 0],
/// #     [0, 0, 0, 2, 0],
/// # ];
/// # let (width, height) = (grid.len(), grid[0].len());
/// #
/// # let neighborhood = ManhattanNeighborhood::new(width, height);
/// #
/// # const COST_MAP: [isize; 3] = [1, 10, -1];
/// #
/// # fn cost_fn<'a>(grid: &'a [[usize; 5]; 5]) -> impl 'a + FnMut(Point) -> isize {
/// #     move |(x, y)| COST_MAP[grid[y][x]]
/// # }
/// #
/// let start = (0, 0);
/// let goal = (2, 0);
///
/// let path = a_star_search(
///     |point| neighborhood.get_all_neighbors(point),
///     cost_fn(&grid),
///     start,
///     goal,
///     |point| neighborhood.heuristic(point, goal),
/// );
///
/// assert!(path.is_none());
/// ```
///
/// ## Solid Goals
/// It is possible to calculate the shortest Path to for example a Wall and other non-walkable
/// Nodes using this function. To do that, simply supply a Function to the `is_walkable` Parameter
/// that returns `false` for Nodes that should not be used as part of an actual Path. If there are
/// no such Nodes in the Graph, `is_walkable` may simply be set to `|_| true`.  
/// In the case that a Path to a non-walkable Goal is requested, the neighbor of that Goal with the
/// shortest Path from the Start is returned, if any is reachable. "neighbor" in this context is
/// a Node for which `get_all_neighbors` contains the Goal.
///
/// ## Arguments
/// - `get_all_neighbors` - a Function that takes a Node and returns all other Nodes reachable from that Node.
///     The returned value is the `Id` of the neighbor.
/// - `get_cost` - a Function that takes a Node and returns the Cost required to walk across that Node.
///     Negative values indicate Nodes that cannot be walked across.
/// - `start` - the starting Node
/// - `goal` - the Goal that this function is supposed to search for
/// - `heuristic` - the Heuristic Function of the A* Algorithm
///
/// ## Returns
/// the Path, if one was found, or None if the `goal` is unreachable.
/// The first Node in the Path is always the `start` and the last is the `goal`
pub fn a_star_search<Id: Copy + Eq + Hash, NeighborIter: Iterator<Item = Id>>(
	mut get_all_neighbors: impl FnMut(Id) -> NeighborIter,
	mut get_cost: impl FnMut(Id) -> isize,
	start: Id,
	goal: Id,
	mut heuristic: impl FnMut(Id) -> Cost,
) -> Option<Path<Id>> {
	if start == goal {
		return Some(Path::new(vec![start, start], 0));
	}
	let mut visited = HashMap::new();
	let mut next = vec![(start, 0)];
	visited.insert(start, (0, start));

	'search: while let Some((current_id, _)) = next.pop() {
		if current_id == goal {
			break 'search;
		}
		let current_cost = visited[&current_id].0;

		let delta_cost = get_cost(current_id);
		if delta_cost < 0 {
			continue;
		}
		let delta_cost = delta_cost as usize;

		for other_id in get_all_neighbors(current_id) {
			let other_cost = current_cost + delta_cost;

			if get_cost(other_id) < 0 && other_id != goal {
				continue;
			}

			let heuristic = heuristic(other_id);

			if let Some(&(prev_cost, _)) = visited.get(&other_id) {
				if prev_cost > other_cost {
					next.retain(|&(id, _)| id != other_id);
				}
			}

			if !visited.contains_key(&other_id) || visited[&other_id].0 > other_cost {
				ordered_insert(
					&mut next,
					(other_id, other_cost + heuristic),
					|&(_, cost)| cost,
				);
				visited.insert(other_id, (other_cost, current_id));
			}
		}
	}

	if !visited.contains_key(&goal) {
		return None;
	}

	let steps = {
		let mut steps = vec![];
		let mut current = goal;

		while current != start {
			steps.push(current);
			let (_, prev) = visited[&current];
			current = prev;
		}
		steps.push(start);
		steps.reverse();
		steps
	};

	Some(Path::new(steps, visited[&goal].0))
}
