//! A Module with some utilities for working with NodeIDs

/// The Type used to reference a Node in the abstracted Graph
pub type NodeID = u32;

/// A convenience type for a [`HashMap`](hashbrown::HashMap) using NodeIDs as the key
pub type NodeIDMap<V> = hashbrown::HashMap<NodeID, V>;

/// A convenience type for a [`HashSet`](hashbrown::HashSet) with NodeIDs
pub type NodeIDSet = hashbrown::HashSet<NodeID>;
