#![allow(unused_imports, clippy::ptr_arg)]

use hierarchical_pathfinding::prelude::*;
use rand::prelude::*;
use rayon::prelude::*;
use std::collections::HashMap;
use std::io::Write;
use std::time::{Duration, Instant};

#[derive(Clone, Debug)]
struct Measurement {
    min: u128,
    max: u128,
    sum: u128,
    count: u128,
}
impl Measurement {
    fn new() -> Self {
        Measurement {
            min: u128::MAX,
            max: 0,
            sum: 0,
            count: 0,
        }
    }
    fn add(&mut self, time: Duration) {
        let ns = duration_as_nanos(time);
        self.min = self.min.min(ns);
        self.max = self.max.max(ns);
        self.sum += ns;
        self.count += 1;
    }
}
impl std::fmt::Display for Measurement {
    fn fmt(&self, fmt: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // let min = self.min as f64 / 1_000_000.0;
        // let max = self.max as f64 / 1_000_000.0;
        let avg = self.sum as f64 / 1_000_000.0 / self.count as f64;
        //                         ms  us  ns
        write!(fmt, "{:>9.2}", avg)
    }
}
#[derive(Debug)]
struct BenchResult {
    cache_gen: Measurement,
    cache_path: Measurement,
    cache_path_unreachable: Measurement,
    cache_modify: Measurement,
    a_star: Measurement,
    a_star_unreachable: Measurement,
    cost_factor: f64,
}
impl std::fmt::Display for BenchResult {
    fn fmt(&self, fmt: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let factor = (self.cost_factor - 1.0) * 100.0;
        write!(
            fmt,
            " {} | {} | {} | {} | {} | {} | {:2.1}%",
            self.a_star,
            self.cache_gen,
            self.cache_path,
            self.cache_modify,
            self.cache_path_unreachable,
            self.a_star_unreachable,
            factor
        )
    }
}
type Grid = Vec<Vec<isize>>;
type Point = (usize, usize);

fn cost_fn(grid: &Grid) -> impl '_ + Fn(Point) -> isize {
    move |(x, y)| grid[y][x]
}

fn main() {
    let mut rng = StdRng::from_entropy();

    let mut output = std::fs::File::create("target/output.txt").unwrap();
    writeln!(
        output,
        " {:>6} | {:^9} | {:>10} | {:>9} | {:>9} | {:>9} | {:>9} | {:>9} | {:>9} | cost factor",
        "grid", "area", "config", "a_star", "generate", "find", "modify", "!find", "!a_star",
    )
    .unwrap();

    for size in [16, 128, 1024].iter().copied() {
        let mut grid = vec![vec![1; size]; size];
        let start = (1, 1);
        let end = size - 2;
        let goal = (end, end);

        // ========== empty grid ==========
        run_benches(&mut output, "empty", &grid, start, goal);

        // ========== snake grid ==========
        grid.par_iter_mut().enumerate().for_each(|(y, row)| {
            if y % 2 != 0 {
                row.fill(-1);
                if y % 4 == 1 {
                    row[0] = 1;
                } else if y % 4 == 3 {
                    row[size - 1] = 1;
                }
            }
        });
        run_benches(&mut output, "snake", &grid, (end, 0), (end, size - 1));

        // random grid
        grid.par_iter_mut().for_each(|row| {
            let mut rng = StdRng::from_entropy();
            row.fill_with(|| rng.gen_range(-5..10));
        });
        let mut start = (0, 0);
        let mut goal = (0, 0);
        let mut possible = false;
        while !possible {
            start = (rng.gen_range(0..size), rng.gen_range(0..size / 5));
            goal = (
                rng.gen_range(0..size),
                size * 4 / 5 + rng.gen_range(0..size / 5),
            );
            possible = a_star_search(
                &ManhattanNeighborhood::new(size, size),
                |_| true,
                cost_fn(&grid),
                start,
                goal,
            )
            .is_some();
        }

        run_benches(&mut output, "random", &grid, start, goal);
    }
}

fn run_benches(output: &mut std::fs::File, name: &str, grid: &Grid, start: Point, goal: Point) {
    let size = grid.len();

    for (config_name, config) in CONFIGS.iter() {
        let result = run_bench(grid, *config, start, goal);
        writeln!(
            output,
            " {:>6} | {:^9} | {:>10} |{}",
            name,
            format!("{0}x{0}", size),
            config_name,
            result
        )
        .unwrap();
    }
    output.flush().unwrap();
}

macro_rules! measure {
    ($target: ident, $code: expr) => {{
        let time_start = Instant::now();
        let result = $code;
        let time_end = Instant::now();
        $target.add(time_end - time_start);
        result
    }};
}

fn run_bench(grid: &Grid, config: PathCacheConfig, start: Point, goal: Point) -> BenchResult {
    let mut cache_gen = Measurement::new();
    let mut cache_path = Measurement::new();
    let mut cache_modify = Measurement::new();
    let mut cache_path_unreachable = Measurement::new();
    let mut a_star = Measurement::new();
    let mut a_star_unreachable = Measurement::new();
    let mut cost_factor = 0.0;

    let size = grid.len();
    let runs = 4096 / size;
    let neigh = ManhattanNeighborhood::new(size, size);
    let mut get_cost = cost_fn(grid);

    let mut grid_unreachable = grid.clone();
    let barrier_y = goal.1 - 1;
    grid_unreachable[barrier_y].fill(-1);
    let changes = (0..size).map(|x| (x, barrier_y)).collect::<Vec<_>>();
    let mut unreachable_fn = cost_fn(&grid_unreachable);

    for _ in 0..runs {
        let a_star_path = measure!(
            a_star,
            a_star_search(&neigh, |_| true, &mut get_cost, start, goal)
        );
        assert!(a_star_path.is_some());

        let mut path_cache = measure!(
            cache_gen,
            PathCache::new_parallel((size, size), &get_cost, neigh, config)
        );

        let path = measure!(cache_path, path_cache.find_path(start, goal, &mut get_cost));
        if path.is_none() {
            let mut out_file = std::fs::File::create("cache.txt").unwrap();
            writeln!(out_file, "path_cache: {:#?}", path_cache).unwrap();
            let mut out_file = std::fs::File::create("grid.txt").unwrap();
            for row in grid {
                for cell in row {
                    write!(out_file, "{}", if *cell < 0 { '#' } else { ' ' }).unwrap();
                }
                writeln!(out_file).unwrap();
            }
        }
        assert!(path.is_some());

        let cost = path.unwrap().cost();
        let a_cost = a_star_path.unwrap();
        cost_factor += cost as f64 / a_cost as f64;

        measure!(
            cache_modify,
            path_cache.tiles_changed(&changes, &mut unreachable_fn)
        );

        let path = measure!(
            cache_path_unreachable,
            path_cache.find_path(start, goal, &mut unreachable_fn)
        );
        assert!(path.is_none());

        let a_star_path = measure!(
            a_star_unreachable,
            a_star_search(&neigh, |_| true, &mut unreachable_fn, start, goal)
        );
        assert!(a_star_path.is_none());
    }

    cost_factor /= runs as f64;

    BenchResult {
        cache_gen,
        cache_path,
        cache_path_unreachable,
        cache_modify,
        a_star,
        a_star_unreachable,
        cost_factor,
    }
}

const DEF: PathCacheConfig = PathCacheConfig {
    chunk_size: 8,
    cache_paths: true,
    a_star_fallback: true,
    perfect_paths: false,
};
const CONFIGS: [(&str, PathCacheConfig); 6] = [
    (
        "chunks_4",
        PathCacheConfig {
            chunk_size: 4,
            ..DEF
        },
    ),
    ("default", DEF),
    (
        "chunks_16",
        PathCacheConfig {
            chunk_size: 16,
            ..DEF
        },
    ),
    (
        "chunks_32",
        PathCacheConfig {
            chunk_size: 32,
            ..DEF
        },
    ),
    (
        "chunks_64",
        PathCacheConfig {
            chunk_size: 64,
            ..DEF
        },
    ),
    (
        "chunks_128",
        PathCacheConfig {
            chunk_size: 128,
            ..DEF
        },
    ),
];

fn duration_as_nanos(d: Duration) -> u128 {
    d.as_secs() as u128 * 1_000_000_000 + d.subsec_nanos() as u128
    //                    s  ms  us  ns
}

// a_star copy

#[derive(PartialEq, Eq)]
pub struct HeuristicElement<Id>(pub Id, pub usize, pub usize);
impl<Id: Eq> PartialOrd for HeuristicElement<Id> {
    fn partial_cmp(&self, rhs: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(rhs))
    }
}
impl<Id: Eq> Ord for HeuristicElement<Id> {
    fn cmp(&self, rhs: &Self) -> std::cmp::Ordering {
        rhs.2.cmp(&self.2)
    }
}

use std::cmp::Ordering;
use std::collections::BinaryHeap;
type PointMap<V> = std::collections::HashMap<Point, V, fnv::FnvBuildHasher>;

pub fn a_star_search<N: Neighborhood>(
    neighborhood: &N,
    mut valid: impl FnMut(Point) -> bool,
    mut get_cost: impl FnMut(Point) -> isize,
    start: Point,
    goal: Point,
) -> Option<usize> {
    if get_cost(start) < 0 {
        return None;
    }
    if start == goal {
        return Some(0);
    }
    let mut visited = PointMap::default();
    let mut next = BinaryHeap::new();
    next.push(HeuristicElement(start, 0, 0));
    visited.insert(start, (0, start));

    let mut all_neighbors = vec![];

    while let Some(HeuristicElement(current_id, current_cost, _)) = next.pop() {
        if current_id == goal {
            break;
        }
        match current_cost.cmp(&visited[&current_id].0) {
            Ordering::Greater => continue,
            Ordering::Equal => {}
            Ordering::Less => panic!("Binary Heap failed"),
        }

        let delta_cost = get_cost(current_id);
        if delta_cost < 0 {
            continue;
        }
        let other_cost = current_cost + delta_cost as usize;

        all_neighbors.clear();
        neighborhood.get_all_neighbors(current_id, &mut all_neighbors);
        for &other_id in all_neighbors.iter() {
            if !valid(other_id) {
                continue;
            }
            if get_cost(other_id) < 0 && other_id != goal {
                continue;
            }

            let mut needs_visit = true;
            if let Some((prev_cost, prev_id)) = visited.get_mut(&other_id) {
                if *prev_cost > other_cost {
                    *prev_cost = other_cost;
                    *prev_id = current_id;
                } else {
                    needs_visit = false;
                }
            } else {
                visited.insert(other_id, (other_cost, current_id));
            }

            if needs_visit {
                let heuristic = neighborhood.heuristic(other_id, goal);
                next.push(HeuristicElement(
                    other_id,
                    other_cost,
                    other_cost + heuristic,
                ));
            }
        }
    }

    if !visited.contains_key(&goal) {
        return None;
    }

    let _steps = {
        let mut steps = vec![];
        let mut current = goal;

        while current != start {
            steps.push(current);
            let (_, prev) = visited[&current];
            current = prev;
        }
        steps.push(start);
        steps.reverse();
        steps
    };

    Some(visited[&goal].0)
}
