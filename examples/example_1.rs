use std::sync::mpsc;
use std::thread;
use std::vec;

use rusty_planner::dstar_lite;
use rusty_planner::planner;

struct Graph {
    moves: i32
}

///
/// Simple example using a basic graph.
///
impl planner::ProblemSpace for Graph {
    type State = i32;
    type Iter = vec::IntoIter<(Self::State, f64)>;

    fn heuristic(&self, _: &Self::State, _: &Self::State) -> f64 { 0.0 }

    fn succ(&self, state: &Self::State) -> Self::Iter {
        match (*state, self.moves) {
            (0, _) => vec![(1, 1.0)].into_iter(),
            (1, _) => vec![(2, 1.0), (3, 5.0)].into_iter(),
            (2, 0) => vec![(4, 1.0)].into_iter(),
            (2, _) => vec![(4, 10.0)].into_iter(),
            (3, _) => vec![(4, 2.0)].into_iter(),
            _ => vec![].into_iter()
        }
    }

    fn pred(&self, state: &Self::State) -> Self::Iter {
        match (*state, self.moves) {
            (1, _) => vec![(0, 1.0)].into_iter(),
            (2, _) => vec![(1, 1.0)].into_iter(),
            (3, _) => vec![(1, 2.0)].into_iter(),
            (4, 0) => vec![(2, 1.0), (3, 5.0)].into_iter(),
            (4, _) => vec![(2, 10.0), (3, 5.0)].into_iter(),
            _ => vec![].into_iter()
        }
    }

    fn update(&mut self, _: &Self::State) {
        self.moves += 1;
    }
}

/// Simple callback printing the path.
fn callback(path: Vec<i32>) {
    print!("start");
    for x in &path {
        print!(" -> {}", x);
    }
    print!("\n");
}

/// Main.
fn main() {
    // Channel so we can communicate with the planner.
    let (tx, rx) = mpsc::channel();
    // The problem space / environment.
    let mut graph = Graph { moves: 0 };

    // Start the planner.
    let plnr = thread::spawn(move || {
        dstar_lite::solve(&mut graph, 0, 4, rx, callback);
    });

    // update state 2, move robot to state 1.
    tx.send((2, 1)).unwrap();
    // signal the planner that we've reached the goal.
    tx.send((-1, 4)).unwrap();
    plnr.join().unwrap();
}