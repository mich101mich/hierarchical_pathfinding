//! A Module with some utilities for working with NodeIDs

/// The Type used to reference a Node in the abstracted Graph
pub type NodeID = u32;
use std::hash::{BuildHasherDefault, Hasher};

/// A specialized [`HashMap`](std::collections::HashMap) for NodeIDs with a faster Hasher
pub type NodeIDMap<V> = std::collections::HashMap<NodeID, V, BuildHasherDefault<NodeIDHasher>>;
/// A specialized [`HashSet`](std::collections::HashSet) for NodeIDs with a faster Hasher
pub type NodeIDSet = std::collections::HashSet<NodeID, BuildHasherDefault<NodeIDHasher>>;

/// A [`Hasher`](Hasher) specialized on NodeIDs
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct NodeIDHasher(u64);

impl Hasher for NodeIDHasher {
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
