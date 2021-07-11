//! A Module with some utilities for working with NodeIDs

/// The Type used to reference a Node in the abstracted Graph
pub type NodeID = u32;

/// A specialized [`HashMap`](std::collections::HashMap) for NodeIDs with a faster Hasher
pub type NodeIDMap<V> = hashbrown::HashMap<NodeID, V>;

/// A specialized [`HashSet`](std::collections::HashSet) for NodeIDs with a faster Hasher
pub type NodeIDSet = hashbrown::HashSet<NodeID>;
