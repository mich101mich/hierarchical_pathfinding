extern crate hierarchical_pathfinding;
use env_logger::Env;

use criterion::{criterion_group, criterion_main, BatchSize, Criterion};

use hierarchical_pathfinding::prelude::*;
use log::warn;

#[derive(Copy, Clone, Debug)]
pub struct Tile {
    cost: isize,
}

#[derive(Clone)]
struct Map {
    tiles: Vec<Tile>,
    width: usize,
    height: usize,
}

impl Map {
    pub fn new(width: usize, height: usize) -> Self {
        let tile_count = width * height;
        Map {
            tiles: vec![Tile { cost: 1 }; tile_count],
            width,
            height,
        }
    }

    pub fn new_random(width: usize, height: usize) -> Self {
        use nanorand::{Rng, WyRand};

        let tile_count = width * height;
        let mut tiles = Vec::with_capacity(tile_count);
        let mut rng = WyRand::new_seed(4);
        for _ in 0..tile_count {
            tiles.push(Tile {
                cost: rng.generate_range(-1_isize..8),
            });
        }
        Map {
            tiles,
            width,
            height,
        }
    }

    #[allow(unused)]
    pub fn set_cost(&mut self, x: usize, y: usize, cost: isize) {
        let pos = self.get_tile_index(x, y);
        if let Some(pos) = pos {
            self.tiles[pos].cost = cost;
        }
    }

    fn get_tile_cost(&self, x: usize, y: usize) -> isize {
        let index = self.get_tile_index(x, y).unwrap();
        self.tiles[index].cost
    }

    fn get_tile_index(&self, x: usize, y: usize) -> Option<usize> {
        if x >= self.width || y >= self.height {
            // Index out of bounds
            return None;
        }

        Some(x + y * self.width)
    }

    fn cost_fn(&self) -> impl '_ + Fn((usize, usize)) -> isize {
        move |(x, y)| self.get_tile_cost(x, y)
    }
}

#[allow(unused)]
// Setup logging output
fn init() {
    let env = Env::default()
        .filter_or("MY_LOG_LEVEL", "trace") // Change this from debug to trace to enable more in-depth timings.
        .write_style_or("MY_LOG_STYLE", "auto");

    env_logger::init_from_env(env);
    let _ = env_logger::builder().is_test(true).try_init();
}

fn bench_create_pathcache(c: &mut Criterion) {
    let mut group = c.benchmark_group("Create PathCache");
    group.sample_size(10);

    // Log to stdout
    init();

    let chunk_sizes = [32];
    let map_sizes = [128, 1024];

    for map_size in map_sizes {
        let (width, height) = (map_size, map_size);
        let map = Map::new(width, height);

        for chunk_size in chunk_sizes {
            #[cfg(feature = "parallel")]
            {
                let id = format!(
                    "Create cache, Uniform map, Parallel, Map Size: ({}, {}), Cache Size: {}",
                    width, height, chunk_size
                );
                group.bench_function(&id, |b| {
                    b.iter(|| {
                        PathCache::new(
                            (width, height),
                            map.cost_fn(),
                            MooreNeighborhood::new(width, height), //
                            PathCacheConfig::with_chunk_size(chunk_size),
                        )
                    })
                });
            }

            let id = format!(
                "Create cache, Uniform map, Single Threaded, Map Size: ({}, {}), Cache Size: {}",
                width, height, chunk_size
            );
            group.bench_function(&id, |b| {
                b.iter(|| {
                    PathCache::new_with_fn_mut(
                        (width, height),
                        map.cost_fn(),
                        MooreNeighborhood::new(width, height),
                        PathCacheConfig::with_chunk_size(chunk_size),
                    )
                })
            });
        }
    }

    // Large random map
    let (width, height) = (1024, 1024);
    let map = Map::new_random(width, height);
    let chunk_size = 32;

    #[cfg(target_os = "windows")]
    warn!("For some reason, the Create PathCache/Large Random Map... benchmark runs significantly slower on Windows than when running the same code in it's own binary.");
    #[cfg(feature = "parallel")]
    {
        let id = format!(
            "Create cache, Large Random Map, Parallel, Map Size: ({}, {}), Cache Size: {}",
            width, height, chunk_size
        );
        group.bench_function(&id, |b| {
            b.iter(|| {
                PathCache::new(
                    (width, height),
                    map.cost_fn(),
                    MooreNeighborhood::new(width, height),
                    PathCacheConfig::with_chunk_size(chunk_size),
                )
            })
        });
    }
    let id = format!(
        "Create cache, Large Random Map, Single Threaded, Map Size: ({}, {}), Cache Size: {}",
        width, height, chunk_size
    );
    group.bench_function(&id, |b| {
        b.iter(|| {
            PathCache::new_with_fn_mut(
                (width, height),
                map.cost_fn(),
                ManhattanNeighborhood::new(width, height),
                PathCacheConfig::with_chunk_size(chunk_size),
            )
        })
    });
}

fn bench_update_pathcache(c: &mut Criterion) {
    let mut group = c.benchmark_group("Update PathCache");
    group.measurement_time(std::time::Duration::from_secs(60));
    // init();

    // Create our map
    let (width, height) = (1024, 1024);
    let mut map = Map::new_random(width, height);
    let chunk_size = 32;
    let pathcache = PathCache::new(
        (width, height),
        map.cost_fn(),
        MooreNeighborhood::new(width, height),
        PathCacheConfig::with_chunk_size(chunk_size),
    );

    // Put a solid wall across our map
    let mut changed = Vec::with_capacity(width);
    for x in 0..width {
        map.set_cost(x, 8, -1);
        changed.push((x, 8));
    }
    #[cfg(feature = "parallel")]
    {
        let id = format!(
            "Update cache, Large Random Map, Parallel, Map Size: ({}, {}), Cache Size: {}",
            width, height, chunk_size
        );
        group.bench_function(&id, |b| {
            // clone on every iteration, so we aren't updaing the same pathcache twice.
            b.iter_batched_ref(
                || pathcache.clone(),
                |cache| cache.tiles_changed(&changed, map.cost_fn()),
                BatchSize::SmallInput,
            )
        });
    }

    group.sample_size(40);

    let id = format!(
        "Update cache, Large Random Map, Single Threaded, Map Size: ({}, {}), Cache Size: {}",
        width, height, chunk_size
    );
    group.bench_function(&id, |b| {
        b.iter_batched_ref(
            || pathcache.clone(),
            |cache| cache.tiles_changed_with_fn_mut(&changed, map.cost_fn()),
            BatchSize::SmallInput,
        )
    });
}

fn bench_get_path(c: &mut Criterion) {
    let mut group = c.benchmark_group("Get Path");

    for (size, iterations, name, start, goal) in [
        (128, 100, "Medium", (0, 0), (127, 127)),
        (256, 50, "Medium+", (1, 1), (200, 250)),
        (512, 20, "Medium++", (20, 30), (500, 400)),
        (1024, 10, "Large", (40, 90), (900, 600)),
    ] {
        group.sample_size(iterations);

        // uniform map
        let map = Map::new(size, size);
        let neighborhood = MooreNeighborhood::new(size, size);
        let chunk_size = 32;
        let pathcache = PathCache::new(
            (size, size),
            map.cost_fn(),
            neighborhood,
            PathCacheConfig::with_chunk_size(chunk_size),
        );
        let id = format!(
            "Get Single Path, {} Uniform Map, Map Size: ({}, {}), Cache Size: {}",
            name, size, size, chunk_size
        );
        #[cfg(feature = "log")]
        {
            log::trace!("");
            log::trace!("{}", id);
            log::trace!("");
        }
        group.bench_function(&id, |b| {
            b.iter(|| pathcache.find_path(start, goal, map.cost_fn()))
        });

        // a_star comparison
        let id = format!(
            "Get Single Path A*, {} Uniform Map, Map Size: ({}, {})",
            name, size, size
        );
        group.bench_function(&id, |b| {
            b.iter(|| a_star_search(&neighborhood, |_| true, map.cost_fn(), start, goal))
        });

        // random map
        let map = Map::new_random(size, size);
        let pathcache = PathCache::new(
            (size, size),
            map.cost_fn(),
            neighborhood,
            PathCacheConfig::with_chunk_size(chunk_size),
        );
        let id = format!(
            "Get Single Path, {} Random Map, Map Size: ({}, {}), Cache Size: {}",
            name, size, size, chunk_size
        );
        #[cfg(feature = "log")]
        {
            log::trace!("");
            log::trace!("{}", id);
            log::trace!("");
        }
        group.bench_function(&id, |b| {
            b.iter(|| pathcache.find_path(start, goal, map.cost_fn()))
        });

        // a_star comparison
        let id = format!(
            "Get Single Path A*, {} Random Map, Map Size: ({}, {})",
            name, size, size
        );
        group.bench_function(&id, |b| {
            b.iter(|| a_star_search(&neighborhood, |_| true, map.cost_fn(), start, goal))
        });
    }
}

criterion_group!(
    benches,
    bench_create_pathcache,
    bench_update_pathcache,
    bench_get_path
);
criterion_main!(benches);

use std::cmp::Ordering;
use std::collections::BinaryHeap;
type Point = (usize, usize);

pub fn a_star_search<N: Neighborhood>(
    neighborhood: &N,
    mut valid: impl FnMut(Point) -> bool,
    mut get_cost: impl FnMut(Point) -> isize,
    start: Point,
    goal: Point,
) -> Option<Vec<Point>> {
    if get_cost(start) < 0 {
        return None;
    }
    if start == goal {
        return Some([start, start].into());
    }
    let mut visited = hashbrown::HashMap::<_, _>::default();
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

    let steps = {
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

    Some(steps)
}

type Cost = usize;
#[derive(PartialEq, Eq)]
pub struct HeuristicElement<Id>(pub Id, pub Cost, pub Cost);
impl<Id: Eq> PartialOrd for HeuristicElement<Id> {
    fn partial_cmp(&self, rhs: &Self) -> Option<Ordering> {
        Some(self.cmp(rhs))
    }
}
impl<Id: Eq> Ord for HeuristicElement<Id> {
    fn cmp(&self, rhs: &Self) -> Ordering {
        rhs.2.cmp(&self.2)
    }
}
