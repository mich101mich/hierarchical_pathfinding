use crate::{generics::Path, NodeID, Point};
use std::collections::HashMap;

#[derive(Clone, Debug)]
pub struct Node {
	pub id: NodeID,
	pub pos: Point,
	pub edges: HashMap<NodeID, Path<Point>>,
}

impl Node {
	pub fn new(id: NodeID, pos: Point) -> Node {
		Node {
			id,
			pos,
			edges: HashMap::new(),
		}
	}
}
