
# Hierarchical Pathfinding

A Rust crate to find Paths on a Grid using HPA* (Hierarchical Pathfinding A*) and Hierarchical Dijkstra.

[![Build Status](https://travis-ci.org/mich101mich/hierarchical_pathfinding.svg?branch=master)](https://travis-ci.org/mich101mich/hierarchical_pathfinding)
Tests: [![CircleCI](https://circleci.com/gh/mich101mich/hierarchical_pathfinding.svg?style=svg)](https://circleci.com/gh/mich101mich/hierarchical_pathfinding)

## Status: not quite finished
- cache creation and pathfinding fully functional
- missing: cache update when the grid changes

## Description
Provides a faster method of finding Paths on a Grid-like structure than regular Pathfinding algorithms. This is achieved by caching segments of Paths that form a higher order Node Graph. Finding a Path in that Graph is considerably faster than traversing all the tiles of the Grid individually, at the cost of producing Paths that are _slightly_ worse than the optimal Path.

### Advantages
- Finding a Path is a lot faster compared to regular algorithms
- It is always correct: A Path is found **if and only if** it exists
  - This means that Hierarchical Pathfinding can be used as Heuristic to check if a Path exists and how long it will roughly be (upper bound)

### Disadvantages
- Paths are slightly worse (negligible in most cases)
- Creating the cache takes time (only happens once at the start)
- Changes to the Grid require updating the cache
  - Whenever a Tile within a Chunk changes, that entire Chunk needs to recalculate its Paths. Performance depends on Chunk size (configurable) and the number of Nodes in a Chunk
