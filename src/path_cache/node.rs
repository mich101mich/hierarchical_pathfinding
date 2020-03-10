use super::path_segment::PathSegment;
use crate::{node_id::*, NodeID, Point};

#[derive(Clone, Debug)]
pub struct Node {
	pub id: NodeID,
	pub pos: Point,
	pub walk_cost: isize,
	pub edges: NodeIDMap<PathSegment>,
}

impl Node {
	pub fn new(id: NodeID, pos: Point, walk_cost: isize) -> Node {
		Node {
			id,
			pos,
			walk_cost,
			edges: node_id_map(),
		}
	}
}
