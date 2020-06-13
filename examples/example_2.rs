use std::vec::IntoIter;

use rusty_planner::any_dyn_astar;
use rusty_planner::planner;

struct Robots {
    start: (i32, i32),
    goal: (i32, i32),
}

///
/// Simple robotic example. Robot can move diagonally or follow the edges of a 10x10 field.
///
impl planner::ProblemSpace for Robots {
    type State = (i32, i32);
    type Iter = IntoIter<((i32, i32), f64)>;

    /// Euclidean distance between 2 points/states.
    fn heuristic(&self, p: &Self::State, q: &Self::State) -> f64 {
        ((q.0 - p.0).pow(2) as f64 + (q.1 - p.1).pow(2) as f64).sqrt()
    }

    /// Defines how robot can move around in 10x10 field.
    fn succ(&self, state: &Self::State) -> Self::Iter {
        let mut v = vec![];
        let (x, y) = *state;
        if x < 10 && y < 10 && x == y {
            v.push(((x + 1, y + 1), 1.0));
        }
        if x < 10 && y == 0 {
            v.push(((x + 1, y), 0.5));
        } else if x == 10 && y < 10 {
            v.push(((x, y + 1), 0.8));
        }
        v.into_iter()
    }

    /// Inverse of succ.
    fn pred(&self, state: &Self::State) -> Self::Iter {
        let mut v = vec![];
        let (x, y) = *state;
        if x > 0 && y > 0 && x == y {
            v.push(((x - 1, y - 1), 1.0));
        }
        if x > 0 && y == 0 {
            v.push(((x - 1, y), 0.5));
        } else if x == 10 && y > 0 {
            v.push(((x, y - 1), 0.8));
        }
        v.into_iter()
    }
}

/// Just print the path the robots takes...
fn callback(path: Vec<(i32, i32)>) {
    for x in &path {
        println!("Robot @ ({}, {})", x.0, x.1);
    }
}

/// Main.
fn main() {
    let robots = Robots { start: (0, 0), goal: (10, 10) };
    any_dyn_astar::solve(&robots, robots.start, robots.goal, callback);
}
