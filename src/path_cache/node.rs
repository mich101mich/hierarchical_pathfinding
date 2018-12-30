use crate::{generics::Path, NodeID, Point};
use std::collections::HashMap;

#[derive(Clone, Debug)]
pub struct Node {
	id: NodeID,
	pos: Point,
	edges: HashMap<NodeID, Path<NodeID>>,
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
