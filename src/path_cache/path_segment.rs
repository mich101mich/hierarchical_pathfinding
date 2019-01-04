use crate::{
	generics::{Cost, Path},
	Point,
};

#[derive(Clone, Debug)]
pub enum PathSegment {
	Known(Path<Point>),
	Unknown {
		start: Point,
		end: Point,
		cost: Cost,
		len: usize,
	},
}

use self::PathSegment::*;

impl PathSegment {
	pub fn new(path: Path<Point>, known: bool) -> PathSegment {
		if known {
			Known(path)
		} else {
			Unknown {
				start: path[0],
				end: path[path.len() - 1],
				cost: path.cost,
				len: path.len(),
			}
		}
	}

	pub fn cost(&self) -> Cost {
		match *self {
			Known(ref path) => path.cost,
			Unknown { cost, .. } => cost,
		}
	}

	pub fn len(&self) -> usize {
		match *self {
			Known(ref path) => path.len(),
			Unknown { len, .. } => len,
		}
	}

	pub fn start(&self) -> Point {
		match *self {
			Known(ref path) => path[0],
			Unknown { start, .. } => start,
		}
	}

	pub fn end(&self) -> Point {
		match *self {
			Known(ref path) => path[path.len() - 1],
			Unknown { end, .. } => end,
		}
	}

	pub fn reversed(&self, start_cost: Cost, end_cost: Cost) -> PathSegment {
		match *self {
			Known(ref path) => Known(path.reversed(start_cost, end_cost)),
			Unknown {
				start,
				end,
				len,
				cost,
			} => Unknown {
				start: end,
				end: start,
				cost: cost + end_cost - start_cost,
				len,
			},
		}
	}
}
