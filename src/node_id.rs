//! A Module with some utilities for working with NodeIDs

use super::NodeID;

/// A specialized [`HashMap`](std::collections::HashMap) for NodeIDs with a faster Hasher
pub type NodeIDMap<V> = std::collections::HashMap<NodeID, V, BuildNodeIDHasher>;
/// A specialized [`HashSet`](std::collections::HashSet) for NodeIDs with a faster Hasher
pub type NodeIDSet = std::collections::HashSet<NodeID, BuildNodeIDHasher>;

/// A [`BuildHasher`](std::hash::BuildHasher) specialized on NodeIDs
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BuildNodeIDHasher;

/// A [`Hasher`](std::hash::Hasher) specialized on NodeIDs
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NodeIDHasher(u64);

impl std::hash::BuildHasher for BuildNodeIDHasher {
	type Hasher = NodeIDHasher;
	fn build_hasher(&self) -> NodeIDHasher {
		NodeIDHasher(0)
	}
}
impl std::hash::Hasher for NodeIDHasher {
	/// panics, since only NodeIDs are supposed to be used
	fn write(&mut self, _: &[u8]) {
		unreachable!("This Hasher only works with NodeIDs")
	}
	/// Writes a single NodeID into this hasher.
	fn write_u32(&mut self, id: NodeID) {
		self.0 = id as u64
	}
	fn finish(&self) -> u64 {
		self.0
	}
}

/// create a new [`NodeIDMap`] by calling the [`with_hasher`](std::collections::HashMap::with_hasher) Function
pub fn node_id_map<V>() -> NodeIDMap<V> {
	NodeIDMap::with_hasher(BuildNodeIDHasher)
}
/// create a new [`NodeIDSet`] by calling the [`with_hasher`](std::collections::HashSet::with_hasher) Function
pub fn node_id_set() -> NodeIDSet {
	NodeIDSet::with_hasher(BuildNodeIDHasher)
}

/// create a new [`NodeIDMap`] by calling the [`with_capacity_and_hasher`](std::collections::HashMap::with_capacity_and_hasher) Function
pub fn node_id_map_with_cap<V>(capacity: usize) -> NodeIDMap<V> {
	NodeIDMap::with_capacity_and_hasher(capacity, BuildNodeIDHasher)
}
/// create a new [`NodeIDSet`] by calling the [`with_capacity_and_hasher`](std::collections::HashSet::with_capacity_and_hasher) Function
pub fn node_id_set_with_cap(capacity: usize) -> NodeIDSet {
	NodeIDSet::with_capacity_and_hasher(capacity, BuildNodeIDHasher)
}
