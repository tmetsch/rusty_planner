use std::vec;

use rusty_planner::dstar_lite;
use rusty_planner::planner;

struct Graph {}

///
/// Simple example using a basic graph.
///
impl planner::ProblemSpace for Graph {
    type State = i32;
    type Iter = vec::IntoIter<(i32, f64)>;

    fn heuristic(&self, _: &Self::State, _: &Self::State) -> f64 { 0.0 }

    fn succ(&self, state: &Self::State) -> Self::Iter {
        match *state {
            0 => vec![(1, 1.0)].into_iter(),
            1 => vec![(2, 1.0), (3, 2.0)].into_iter(),
            2 => vec![(4, 1.0)].into_iter(),
            3 => vec![(4, 2.0)].into_iter(),
            _ => vec![].into_iter()
        }
    }

    fn pred(&self, state: &Self::State) -> Self::Iter {
        match *state {
            1 => vec![(0, 1.0)].into_iter(),
            2 => vec![(1, 1.0)].into_iter(),
            3 => vec![(1, 2.0)].into_iter(),
            4 => vec![(2, 1.0), (3, 2.0)].into_iter(),
            _ => vec![].into_iter()
        }
    }
}

/// Main.
fn main() {
    let graph = Graph {};
    dstar_lite::solve(&graph, 0, 4);
}