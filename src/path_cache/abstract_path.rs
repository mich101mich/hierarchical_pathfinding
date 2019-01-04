use super::path_segment::{PathSegment, PathSegment::*};
use crate::{
	generics::{a_star_search, Cost, Path},
	neighbors::Neighborhood,
	Point,
};

use std::fmt::Debug;

pub trait AbstractPath: Iterator<Item = Point> + Debug {
	fn cost(&self) -> Cost;
	fn safe_next(&mut self, get_cost: impl Fn(Point) -> isize) -> Option<Point>;
}

#[derive(Debug)]
pub struct AbstractPathImpl<N: Neighborhood + Debug> {
	neighborhood: Option<N>,
	total_cost: Cost,
	path: Vec<PathSegment>,
	end: Point,
	current_index: (usize, usize),
}

impl<N> AbstractPathImpl<N>
where
	N: Neighborhood + Debug,
{
	pub fn new(start: Point) -> AbstractPathImpl<N> {
		AbstractPathImpl {
			neighborhood: None,
			total_cost: 0,
			path: vec![],
			end: start,
			current_index: (0, 1),
		}
	}

	pub fn from_known_path(path: Path<Point>) -> AbstractPathImpl<N> {
		let end = path[path.len() - 1];
		AbstractPathImpl {
			neighborhood: None,
			total_cost: path.cost,
			path: vec![Known(path)],
			end,
			current_index: (0, 1),
		}
	}

	pub fn from_node(neighborhood: N, node: Point) -> AbstractPathImpl<N> {
		AbstractPathImpl {
			neighborhood: Some(neighborhood),
			total_cost: 0,
			path: vec![],
			end: node,
			current_index: (0, 1),
		}
	}

	pub fn add_path_segment(&mut self, path: PathSegment) -> &mut Self {
		assert!(self.end == path.start(), "Added disconnected PathSegment");
		self.total_cost += path.cost();
		self.end = path.end();
		self.path.push(path);
		self
	}

	pub fn add_path(&mut self, path: Path<Point>) -> &mut Self {
		self.total_cost += path.cost;
		self.end = path[path.len() - 1];
		self.path.push(Known(path));
		self
	}

	pub fn add_node(&mut self, node: Point, cost: Cost, len: usize) -> &mut Self {
		self.path.push(Unknown {
			start: self.end,
			end: node,
			cost,
			len,
		});
		self.total_cost += cost;
		self.end = node;
		self
	}
}

impl<N> AbstractPath for AbstractPathImpl<N>
where
	N: Neighborhood + Debug,
{
	fn cost(&self) -> Cost {
		self.total_cost
	}
	fn safe_next(&mut self, get_cost: impl Fn(Point) -> isize) -> Option<Point> {
		if self.current_index.0 >= self.path.len() {
			return None;
		}
		let mut current = &self.path[self.current_index.0];
		if let Unknown { start, end, .. } = *current {
			let neighborhood = self
				.neighborhood
				.as_ref()
				.expect("Unknown segment in Known Path");

			let path = a_star_search(
				|p| neighborhood.get_all_neighbors(p),
				|p, _| get_cost(p) as usize,
				|p| get_cost(p) >= 0,
				start,
				end,
				|p| neighborhood.heuristic(p, end),
			)
			.unwrap_or_else(|| {
				panic!(
					"Impossible Path marked as Possible: {:?} -> {:?}",
					start, end
				)
			});

			self.path[self.current_index.0] = Known(path);
			current = &self.path[self.current_index.0];

			self.current_index.1 = 1; // paths include start and end, but we are already at start
		}

		if let Known(path) = current {
			let ret = path[self.current_index.1];
			self.current_index.1 += 1;
			if self.current_index.1 >= path.len() {
				self.current_index.0 += 1;
				self.current_index.1 = 0;
			}
			Some(ret)
		} else {
			panic!("how.");
		}
	}
}

impl<N> Iterator for AbstractPathImpl<N>
where
	N: Neighborhood + Debug,
{
	type Item = Point;
	fn next(&mut self) -> Option<Point> {
		if self.current_index.0 >= self.path.len() {
			return None;
		}
		let current = &self.path[self.current_index.0];
		if let Unknown { .. } = *current {
			panic!(
				"Tried calling next() on a Path that is not fully known. Use safe_next instead."
			);
		}

		if let Known(path) = current {
			let ret = path[self.current_index.1];
			self.current_index.1 += 1;
			if self.current_index.1 >= path.len() {
				self.current_index.0 += 1;
				self.current_index.1 = 1;
			}
			Some(ret)
		} else {
			panic!("how.");
		}
	}
}
