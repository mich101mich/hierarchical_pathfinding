extern crate hierarchical_pathfinding;
use env_logger::Env;

use criterion::{criterion_group, criterion_main, Criterion};

use hierarchical_pathfinding::prelude::*;
use log::warn;
use oorandom::Rand32;

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
        let tile_count = width * height;
        let mut tiles = Vec::with_capacity(tile_count);
        let mut rng = Rand32::new(4);
        for _ in 0..tile_count {
            tiles.push(Tile {
                cost: rng.rand_range(0..10) as isize - 1,
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
}

#[allow(unused)]
// Setup logging output
fn init() {
    let env = Env::default()
        .filter_or("MY_LOG_LEVEL", "debug") // Change this from debug to trace to enable more in-depth timings.
        .write_style_or("MY_LOG_STYLE", "always");

    env_logger::init_from_env(env);
    let _ = env_logger::builder().is_test(true).try_init();
}

fn bench_create_patchcache(c: &mut Criterion) {
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
                        PathCache::new_parallel(
                            (width, height),
                            |(x, y)| map.get_tile_cost(x, y),
                            MooreNeighborhood::new(width, height), //
                            PathCacheConfig {
                                chunk_size,
                                ..Default::default()
                            },
                        )
                    })
                });
            }

            #[cfg(not(feature = "parallel"))]
            {
                let id = format!(
                    "Create cache, Single Threaded, Map Size: ({}, {}), Cache Size: {}",
                    width, height, chunk_size
                );
                group.bench_function(&id, |b| {
                    b.iter(|| {
                        PathCache::new(
                            (width, height),
                            |(x, y)| map.get_tile_cost(x, y),
                            MooreNeighborhood::new(width, height),
                            PathCacheConfig {
                                chunk_size,
                                ..Default::default()
                            },
                        )
                    })
                });
            }
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
        group.bench_with_input(
            &id,
            &(map, chunk_size, width, height),
            |b, (map, chunk_size, width, height)| {
                b.iter(|| {
                    PathCache::new_parallel(
                        (*width, *height),
                        |(x, y)| map.get_tile_cost(x, y),
                        MooreNeighborhood::new(*width, *height),
                        PathCacheConfig {
                            chunk_size: *chunk_size,
                            ..Default::default()
                        }, // config
                    )
                })
            },
        );
    }
    #[cfg(not(feature = "parallel"))]
    {
        let id = format!(
            "Create cache, Large Random Map, Single Threaded, Map Size: ({}, {}), Cache Size: {}",
            width, height, chunk_size
        );
        group.bench_function(&id, |b| {
            b.iter(|| {
                PathCache::new(
                    (width, height),
                    |(x, y)| map.get_tile_cost(x, y),
                    ManhattanNeighborhood::new(width, height),
                    PathCacheConfig {
                        chunk_size,
                        ..Default::default()
                    },
                )
            })
        });
    }
}

fn bench_update_patchcache(c: &mut Criterion) {
    let mut group = c.benchmark_group("Update PathCache");
    // init();

    // Create our map
    let (width, height) = (1024, 1024);
    let mut map = Map::new_random(width, height);
    let chunk_size = 32;
    #[cfg(feature = "parallel")]
    let mut pathcache = PathCache::new_parallel(
        (width, height),
        |(x, y)| map.get_tile_cost(x, y),
        MooreNeighborhood::new(width, height),
        PathCacheConfig {
            chunk_size,
            ..Default::default()
        },
    );

    #[cfg(not(feature = "parallel"))]
    let mut pathcache = PathCache::new(
        (width, height),
        |(x, y)| map.get_tile_cost(x, y),
        MooreNeighborhood::new(width, height),
        PathCacheConfig {
            chunk_size,
            ..Default::default()
        },
    );

    // Use this for the parllel bench, so we aren't updaing the same pathcache twice.
    let mut pathcache_clone = pathcache.clone();

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
            b.iter(|| {
                pathcache_clone.tiles_changed_parallel(&changed, |(x, y)| map.get_tile_cost(x, y));
            })
        });
    }

    group.sample_size(40);

    let id = format!(
        "Update cache, Large Random Map, Single Threaded, Map Size: ({}, {}), Cache Size: {}",
        width, height, chunk_size
    );
    group.bench_function(&id, |b| {
        b.iter(|| {
            pathcache.tiles_changed(&changed, |(x, y)| map.get_tile_cost(x, y));
        })
    });
}

fn bench_get_path(c: &mut Criterion) {
    let mut group = c.benchmark_group("Get Path");

    // Medium uniform map
    let (width, height) = (128, 128);
    let map = Map::new(width, height);
    let chunk_size = 32;
    #[cfg(feature = "parallel")]
    let pathcache = PathCache::new_parallel(
        (width, height),
        |(x, y)| map.get_tile_cost(x, y),
        MooreNeighborhood::new(width, height),
        PathCacheConfig {
            chunk_size,
            ..Default::default()
        },
    );
    #[cfg(not(feature = "parallel"))]
    let pathcache = PathCache::new(
        (width, height),
        |(x, y)| map.get_tile_cost(x, y),
        MooreNeighborhood::new(width, height),
        PathCacheConfig {
            chunk_size,
            ..Default::default()
        },
    );
    let id = format!(
        "Get Single Path, Medium Uniform Map, Map Size: ({}, {}), Cache Size: {}",
        width, height, chunk_size
    );
    group.bench_function(&id, |b| {
        b.iter(|| {
            pathcache.find_path((0, 0), (127, 127), |(x, y)| map.get_tile_cost(x, y));
        })
    });

    // Medium random map
    let (width, height) = (128, 128);
    let map = Map::new_random(width, height);
    let chunk_size = 32;

    #[cfg(feature = "parallel")]
    let pathcache = PathCache::new_parallel(
        (width, height),
        |(x, y)| map.get_tile_cost(x, y),
        MooreNeighborhood::new(width, height),
        PathCacheConfig {
            chunk_size,
            ..Default::default()
        },
    );
    #[cfg(not(feature = "parallel"))]
    let pathcache = PathCache::new(
        (width, height),
        |(x, y)| map.get_tile_cost(x, y),
        MooreNeighborhood::new(width, height),
        PathCacheConfig {
            chunk_size,
            ..Default::default()
        },
    );

    let id = format!(
        "Get Single Path, Medium Random Map, Map Size: ({}, {}), Cache Size: {}",
        width, height, chunk_size
    );
    group.bench_function(&id, |b| {
        b.iter(|| {
            pathcache.find_path((0, 0), (127, 127), |(x, y)| map.get_tile_cost(x, y));
        })
    });

    // For large maps, use a smaller sample size so they don't take 30+s per run.
    group.sample_size(10);

    // Large Uniform map
    let (width, height) = (1024, 1024);
    let map = Map::new(width, height);
    let chunk_size = 32;
    #[cfg(feature = "parallel")]
    let pathcache = PathCache::new_parallel(
        (width, height),
        |(x, y)| map.get_tile_cost(x, y),
        MooreNeighborhood::new(width, height),
        PathCacheConfig {
            chunk_size,
            ..Default::default()
        },
    );
    #[cfg(not(feature = "parallel"))]
    let pathcache = PathCache::new(
        (width, height),
        |(x, y)| map.get_tile_cost(x, y),
        MooreNeighborhood::new(width, height),
        PathCacheConfig {
            chunk_size,
            ..Default::default()
        },
    );

    let id = format!(
        "Get Single Path, Large Uniform Map, Map Size: ({}, {}), Cache Size: {}",
        width, height, chunk_size
    );
    group.bench_function(&id, |b| {
        b.iter(|| {
            pathcache.find_path((40, 90), (900, 600), |(x, y)| map.get_tile_cost(x, y));
        })
    });

    // Large Random map
    let (width, height) = (1024, 1024);
    let map = Map::new_random(width, height);
    let chunk_size = 64;
    #[cfg(feature = "parallel")]
    let pathcache = PathCache::new_parallel(
        (width, height),
        |(x, y)| map.get_tile_cost(x, y),
        MooreNeighborhood::new(width, height),
        PathCacheConfig {
            chunk_size,
            ..Default::default()
        },
    );
    #[cfg(not(feature = "parallel"))]
    let pathcache = PathCache::new(
        (width, height),
        |(x, y)| map.get_tile_cost(x, y),
        MooreNeighborhood::new(width, height),
        PathCacheConfig {
            chunk_size,
            ..Default::default()
        },
    );

    let id = format!(
        "Get Single Path, Large Random Map, Map Size: ({}, {}), Cache Size: {}",
        width, height, chunk_size
    );
    group.bench_function(&id, |b| {
        b.iter(|| {
            pathcache.find_path((40, 90), (900, 600), |(x, y)| map.get_tile_cost(x, y));
        })
    });
}

criterion_group!(
    benches,
    bench_create_patchcache,
    bench_update_patchcache,
    bench_get_path
);
criterion_main!(benches);
