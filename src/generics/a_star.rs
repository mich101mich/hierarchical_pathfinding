use super::{ordered_insert, Cost, Path};
use std::collections::HashMap;
use std::hash::Hash;

/// Searches a Graph using the [A* Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm).
///
/// The Generic type Parameter ```Id``` is supposed to uniquely identify a Node in the Graph.
/// This may be a Number, String, a Grid position, ... as long as it can be compared, hashed and copied.
/// Note that it is advised to choose a short representation for the Id, since it will be copied several times.
///
/// ## Examples
/// Basic usage:
/// ```
/// # use hierarchical_pathfinding::generics::a_star_search;
/// // A     B--2--E
/// // |\     
/// // | \    
/// // 1  9   
/// // |   \  
/// // |    \
/// // C--6--D
/// let (A, B, C, D, E) = (0, 1, 2, 3, 4);
/// let cost_matrix: [[i32; 5]; 5] = [
/// //    A,  B,  C,  D,  E
/// 	[-1, -1,  1,  9, -1], // A
/// 	[-1, -1, -1, -1,  2], // B
/// 	[ 1, -1, -1,  6, -1], // C
/// 	[ 9, -1,  6, -1, -1], // D
/// 	[-1,  2, -1, -1, -1], // E
/// ];
/// # fn euclid_distance(a: usize, b: usize) -> usize {
/// # 	  [[0, 1, 1, 2, 2], [1, 0, 2, 1, 1], [1, 2, 0, 1, 3], [2, 1, 1, 0, 2], [2, 1, 3, 2, 0]][a][b]
/// # }
///
/// let result = a_star_search(
/// 	|point| { // get_all_neighbors
/// 		cost_matrix[point]
/// 			.iter()
/// 			.enumerate()
/// 			.filter(|&(_, cost)| *cost != -1)
/// 			.map(|(id, _)| id)
/// 			.collect()
/// 	},
/// 	|a, b| cost_matrix[a][b] as usize, // get_cost
/// 	|_| true, // is_walkable
/// 	A, // start
/// 	D, // goal
/// 	|point| euclid_distance(point, D), // heuristic
/// );
///
/// assert!(result.is_some());
/// let path = result.unwrap();
///
/// assert_eq!(path.path, vec![A, C, D]);
/// assert_eq!(path.cost, 7);
/// ```
///
/// If the Goal cannot be reached, None is returned:
/// ```
/// # use hierarchical_pathfinding::generics::a_star_search;
/// # let (A, B, C, D, E) = (0, 1, 2, 3, 4);
/// # let cost_matrix: [[i32; 5]; 5] = [
/// # //    A,  B,  C,  D,  E
/// #     [-1, -1,  1,  9, -1], // A
/// #     [-1, -1, -1, -1,  2], // B
/// #     [ 1, -1, -1,  6, -1], // C
/// #     [ 9, -1,  6, -1, -1], // D
/// #     [-1,  2, -1, -1, -1], // E
/// # ];
/// # fn euclid_distance(a: usize, b: usize) -> usize {
/// # 	  [[0, 1, 1, 2, 2], [1, 0, 2, 1, 1], [1, 2, 0, 1, 3], [2, 1, 1, 0, 2], [2, 1, 3, 2, 0]][a][b]
/// # }
/// #
/// # let result = a_star_search(
/// #    |point| { // get_all_neighbors
/// #        cost_matrix[point]
/// #            .iter()
/// #            .enumerate()
/// #            .filter(|&(_, cost)| *cost != -1)
/// #            .map(|(id, _)| id)
/// #            .collect()
/// #    },
/// #    |a, b| cost_matrix[a][b] as usize, // get_cost
/// #    |_| true, // is_walkable
/// // ...
///     A, // start
///     E, // goal
/// 	|point| euclid_distance(point, E), // heuristic
/// );
///
/// assert_eq!(result, None);
/// ```
///
/// ## Solid Goals
/// It is possible to calculate the shortest Path to for example a Wall and other non-walkable Nodes using this function.
/// To do that, simply supply a Function to the is_walkable Parameter that returns ```false``` for Nodes that
/// should not be used as part of an actual Path. If there are no such Nodes in the Graph,
/// is_walkable may simply be set to ```|_| true```
///
/// ## Arguments
/// - ```get_all_neighbors``` - a Function that takes a Node and returns all other Nodes reachable from that Node
/// - ```get_cost``` - a Function that takes two Nodes (a, b) and returns the Cost to go from a to b
/// - ```is_walkable``` - a Function that determines if a Node can be walked over. see [Solid Goals](#solid-goals) for more info
/// - ```start``` - the starting Node
/// - ```goal``` - the Goal that this function is supposed to search for
/// - ```heuristic``` - the Heuristic Function of the A* Algorithm
///
/// ## Returns
/// the Path, if one was found, or None if the ```goal``` is unreachable.
/// The first Node in the Path is always the ```start``` and the last is the ```goal```
pub fn a_star_search<Id: Copy + Eq + Hash>(
	get_all_neighbors: impl Fn(Id) -> Vec<Id>,
	get_cost: impl Fn(Id, Id) -> Cost,
	is_walkable: impl Fn(Id) -> bool,
	start: Id,
	goal: Id,
	heuristic: impl Fn(Id) -> Cost,
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

		for other_id in get_all_neighbors(current_id) {
			let other_cost = current_cost + get_cost(current_id, other_id);

			if !is_walkable(other_id) && other_id != goal {
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
