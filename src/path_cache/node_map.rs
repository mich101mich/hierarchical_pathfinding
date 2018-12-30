use super::Node;
use crate::{NodeID, Point};
use std::collections::HashMap;

#[derive(Clone, Debug)]
pub struct NodeMap {
	nodes: HashMap<NodeID, Node>,
	next_id: NodeID,
}

impl NodeMap {
	pub fn new() -> NodeMap {
		NodeMap {
			nodes: HashMap::new(),
			next_id: 0,
		}
	}
	pub fn add_node(&mut self, pos: Point) -> NodeID {
		let id = self.next_id;
		self.next_id += 1;

		let node = Node::new(id, pos);
		self.nodes.insert(id, node);
		id
	}
}

use std::ops::Deref;
impl Deref for NodeMap {
	type Target = HashMap<NodeID, Node>;
	fn deref(&self) -> &HashMap<NodeID, Node> {
		&self.nodes
	}
}
