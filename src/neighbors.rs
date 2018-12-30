//! A crate with the most common Neighborhood and Heuristic functions

use crate::Point;

/// Creates a function that returns the [Von Neumann Neighborhood](https://en.wikipedia.org/wiki/Von_Neumann_neighborhood) of a Point.
/// All Neighbors will be inside [0, width) x [0, height).
///
/// This is useful for the ```get_all_neighbors``` Parameter of several functions in [PathCache](struct.PathCache.html).
///
/// ## Examples
/// Basic usage:
/// ```
/// use hierarchical_pathfinding::neighbors::manhattan_neighbors;
///
/// let get_all_neighbors = manhattan_neighbors(5, 5);
///
/// let mut iter = get_all_neighbors((0, 2));
/// assert_eq!(iter.next(), Some((1, 2)));
/// assert_eq!(iter.next(), Some((0, 1)));
/// assert_eq!(iter.next(), Some((0, 3)));
/// assert_eq!(iter.next(), None);
/// ```
pub fn manhattan_neighbors(
	width: usize,
	height: usize,
) -> impl Fn(Point) -> std::vec::IntoIter<Point> {
	move |(x, y)| {
		let mut neighbors = vec![];
		if x > 0 {
			neighbors.push((x - 1, y));
		}
		if x < width - 1 {
			neighbors.push((x + 1, y));
		}
		if y > 0 {
			neighbors.push((x, y - 1));
		}
		if y < height - 1 {
			neighbors.push((x, y + 1));
		}
		neighbors.into_iter()
	}
}

/// Creates a Function that calculates the Distance between a Point and the provided goal using the Manhattan Metric.
///
/// This is useful for the ```heuristic``` Parameter of [PathCache::find_path](struct.PathCache.html#find_path).
///
/// [The Manhattan Metric](https://en.wikipedia.org/wiki/Taxicab_geometry) (also known as Taxicab Geometry)
/// is the sum of the absolute difference between the x and y coordinates.
///
/// In other words: If an Agent can only move along the 4 Axes, the time required to get
/// from A to B is the Manhattan Metric between A and B.
///
/// ## Examples
/// Basic usage:
/// ```
/// use hierarchical_pathfinding::neighbors::manhattan_heuristic;
///
/// let goal = (3, 1);
/// let heuristic = manhattan_heuristic(goal);
///
/// let start = (0, 0);
///
/// assert_eq!(heuristic(start), 3 + 1);
/// ```
pub fn manhattan_heuristic(goal: Point) -> impl Fn(Point) -> usize {
	move |point| {
		let diff_0 = if goal.0 > point.0 {
			goal.0 - point.0
		} else {
			point.0 - goal.0
		};
		let diff_1 = if goal.1 > point.1 {
			goal.1 - point.1
		} else {
			point.1 - goal.1
		};
		diff_0 + diff_1
	}
}

/// Creates a function that returns the [Moore Neighborhood](https://en.wikipedia.org/wiki/Moore_neighborhood) of a Point.
/// All Neighbors will be inside [0, width) x [0, height).
///
/// This is useful for the get_all_neighbors Parameter of several functions in [PathCache](struct.PathCache.html).
///
/// ## Examples
/// Basic usage:
/// ```
/// use hierarchical_pathfinding::neighbors::moore_neighbors;
///
/// let get_all_neighbors = moore_neighbors(5, 5);
///
/// let mut iter = get_all_neighbors((0, 2));
/// assert_eq!(iter.next(), Some((0, 1)));
/// assert_eq!(iter.next(), Some((0, 3)));
/// assert_eq!(iter.next(), Some((1, 1)));
/// assert_eq!(iter.next(), Some((1, 2)));
/// assert_eq!(iter.next(), Some((1, 3)));
/// assert_eq!(iter.next(), None);
/// ```
pub fn moore_neighbors(width: usize, height: usize) -> impl Fn(Point) -> std::vec::IntoIter<Point> {
	move |(x, y)| {
		let mut neighbors = vec![];
		if x > 0 {
			if y > 0 {
				neighbors.push((x - 1, y - 1));
			}
			neighbors.push((x - 1, y));
			if y < height - 1 {
				neighbors.push((x - 1, y + 1));
			}
		}
		if y > 0 {
			neighbors.push((x, y - 1));
		}
		if y < height - 1 {
			neighbors.push((x, y + 1));
		}
		if x < width - 1 {
			if y > 0 {
				neighbors.push((x + 1, y - 1));
			}
			neighbors.push((x + 1, y));
			if y < height - 1 {
				neighbors.push((x + 1, y + 1));
			}
		}
		neighbors.into_iter()
	}
}

/// Creates a Function that calculates the Distance between a Point and the provided goal using the Maximum Metric.
///
/// This is useful for the ```heuristic``` Parameter of [PathCache::find_path](struct.PathCache.html#find_path).
///
/// [The Maximum Metric](https://en.wikipedia.org/wiki/Chebyshev_distance) (also known Chebyshev distance)
/// is the maximum of the absolute difference between the x and y coordinates.
///
/// In other words: If an Agent can move along the 4 Axes and all 4 Diagonals, the time required to get
/// from A to B is the Maximum Metric between A and B.
///
/// ## Examples
/// Basic usage:
/// ```
/// use hierarchical_pathfinding::neighbors::moore_heuristic;
///
/// let goal = (3, 1);
/// let heuristic = moore_heuristic(goal);
///
/// let start = (0, 0);
///
/// assert_eq!(heuristic(start), 3);
/// ```
pub fn moore_heuristic(goal: Point) -> impl Fn(Point) -> usize {
	move |point| {
		let diff_0 = if goal.0 > point.0 {
			goal.0 - point.0
		} else {
			point.0 - goal.0
		};
		let diff_1 = if goal.1 > point.1 {
			goal.1 - point.1
		} else {
			point.1 - goal.1
		};
		diff_0.max(diff_1)
	}
}
