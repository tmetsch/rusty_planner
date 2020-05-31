extern crate rusty_planner;

use std::vec;

#[test]
fn test_simple_example() {
    struct Example {}

    impl rusty_planner::planner::ProblemSpace for Example {
        type State = i32;
        type Iter = vec::IntoIter<(i32, f64)>;

        fn heuristic(&self, _: &Self::State, _: &Self::State) -> f64 {
            1.0
        }

        fn succ(&self, state: &Self::State) -> Self::Iter {
            match *state {
                0 => vec![(1, 1.0)].into_iter(),
                1 => vec![(2, 1.0), (3, 1.0)].into_iter(),
                2 => vec![(3, 2.0), (4, 1.0)].into_iter(),
                3 => vec![(4, 10.0)].into_iter(),
                _ => vec![].into_iter()
            }
        }

        fn pred(&self, state: &Self::State) -> Self::Iter {
            match *state {
                1 => vec![(0, 1.0)].into_iter(),
                2 => vec![(1, 1.0)].into_iter(),
                3 => vec![(1, 1.0), (2, 2.0)].into_iter(),
                4 => vec![(2, 1.0), (3, 10.0)].into_iter(),
                _ => vec![].into_iter()
            }
        }
    }

    fn callback(path: Vec<i32>) {
        assert_eq!(path, [1, 2, 4]);
    }

    let example = Example {};
    rusty_planner::any_dyn_astar::solve(&example, 0, 4, callback);
    assert_eq!(rusty_planner::dstar_lite::solve(&example, 0, 4), [1, 2, 4]);
}