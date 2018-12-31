
# Hierarchical Pathfinding

A Rust crate to find Paths on a Grid using HPA* (Hierarchical Pathfinding A*) and Hierarchical Dijkstra.

[![Build Status](https://travis-ci.org/mich101mich/hierarchical_pathfinding.svg?branch=master)](https://travis-ci.org/mich101mich/hierarchical_pathfinding)
Tests: [![CircleCI](https://circleci.com/gh/mich101mich/hierarchical_pathfinding.svg?style=svg)](https://circleci.com/gh/mich101mich/hierarchical_pathfinding)

## Introduction
Finding Paths on a Grid using regular A* and Dijkstra is usually a rather expensive Operation,
since every Tile on the Grid is considered a Node, leading to those algorithms having to
store and visit hundreds of Nodes for a fairly short Path. Combined with the fact that Grids
usually have several Paths with the exact same Cost makes a naive implementation of regular
Pathfinding Algorithms rather inefficient.

The idea behind Hierarchical Pathfinding is to improve that Process by collecting sections of
the Grid into Chunks and pre-calculating and caching the cost (and possibly the Paths) of
walking between the different entrances of the Chunk. The final process of finding a Path
between two points can then use that knowledge by treating the Grid as a Graph with Entrances
as Nodes and the cached costs (and Paths) as Edges, resulting in a much smaller Graph that
can be easily searched.

Since the Graph is usually not an exact representation of the Grid, **the resulting Paths will
be slightly worse than the actual best Path**. This is usually not a problem, since the
purpose of Hierarchical Pathfinding is to quickly find the next direction to go in or a
Heuristic for the total Cost of a Path or to determine weather or not a Goal is reachable.
All of these are not affected by the exact Cost or Path. The only time where the actual best
Path would noticeably differ from this crates result is in the case of small Paths of
```Length < 2 * chunk_size```. That is why this implementation calls the regular A* search
after HPA* confirmed the Path to be short. (This behavior can be turned of using the Config).

This crate provides an implementation of a Hierarchical Pathfinding Algorithm for any generic Grid.
Paths can be searched using either A* for a Path to a single Tile, or Dijkstra for searching multiple Targets.
