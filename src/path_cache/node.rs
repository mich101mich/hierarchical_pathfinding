use super::path_segment::PathSegment;
use crate::{NodeID, Point};
use std::collections::HashMap;

#[derive(Clone, Debug)]
pub struct Node {
	pub id: NodeID,
	pub pos: Point,
	pub walk_cost: isize,
	pub edges: HashMap<NodeID, PathSegment>,
}

impl Node {
	pub fn new(id: NodeID, pos: Point, walk_cost: isize) -> Node {
		Node {
			id,
			pos,
			walk_cost,
			edges: HashMap::new(),
		}
	}
}
