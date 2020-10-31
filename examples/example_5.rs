use std::vec;

use rusty_planner::mcts;
use rusty_planner::planner;

const WIDTH: i32 = 5;
const HEIGHT: i32 = 5;

struct Arena {
    obstacles: Vec<(i32, i32)>,
}

impl planner::ProblemSpace for Arena {
    type State = (i32, i32);
    type Iter = vec::IntoIter<(Self::State, f64)>;

    fn heuristic(&self, _: &Self::State, _: &Self::State) -> f64 {
        unimplemented!()
    }

    fn succ(&self, state: &Self::State) -> Self::Iter {
        let mut res = vec![];
        if state.0 < WIDTH && !self.obstacles.contains(&(state.0 + 1, state.1)) {
            // move right.
            res.push(((state.0 + 1, state.1), 1.0));
        }
        if state.1 < HEIGHT && !self.obstacles.contains(&(state.0, state.1 + 1)) {
            // move up.
            res.push(((state.0, state.1 + 1), 1.0));
        }
        if state.0 < WIDTH
            && state.1 < HEIGHT
            && !self.obstacles.contains(&(state.0 + 1, state.1 + 1))
        {
            // move diagonal right & up.
            res.push(((state.0 + 1, state.1 + 1), 0.8));
        }
        res.into_iter()
    }

    fn pred(&self, state: &Self::State) -> Self::Iter {
        let mut res = vec![];
        if state.0 >= 1 && !self.obstacles.contains(&(state.0 - 1, state.1)) {
            // move left.
            res.push(((state.0 - 1, state.1), 1.0));
        }
        if state.1 >= 1 && !self.obstacles.contains(&(state.0, state.1 - 1)) {
            // move down.
            res.push(((state.0, state.1 - 1), 1.0));
        }
        if state.0 >= 1 && state.1 >= 0 && !self.obstacles.contains(&(state.0 - 1, state.1 - 1)) {
            // move diagonal left & down.
            res.push(((state.0 - 1, state.1 - 1), 0.8));
        }
        res.into_iter()
    }
}

/// Simple callback routine.
fn callback(s_0: &(i32, i32)) {
    println!("Current best next step: {:?}", s_0);
}

fn main() {
    let ps = Arena {
        obstacles: vec![(2, 2), (3, 3), (4, 3), (4, 4)],
    };
    let start = (1, 1);
    let goal = (WIDTH, HEIGHT);
    mcts::solve(&ps, start, goal, 3, callback);
}
