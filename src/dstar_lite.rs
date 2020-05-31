use std::collections;

use crate::planner;
use crate::util;

fn key(data: &util::StateData, h: f64) -> (f64, f64) {
    let mut k_0 = data.g;
    if data.rhs + h < k_0 {
        k_0 = data.rhs + h;
    }
    let mut k_1 = data.g;
    if data.rhs > k_1 {
        k_1 = data.rhs;
    }
    (k_0, k_1)
}

fn update_state<PS: planner::ProblemSpace>(
    ps: &PS,
    s: PS::State,
    start: PS::State,
    goal: PS::State,
    data: &mut collections::HashMap<PS::State, util::StateData>,
    open: &mut collections::BinaryHeap<util::HeapEntry<PS::State>>) {
    if !data.contains_key(&s) {
        data.insert(s, util::StateData { rhs: f64::INFINITY, g: f64::INFINITY });
    }
    if s != goal {
        let mut tmp = f64::INFINITY;
        for item in ps.succ(&s) {
            if data.contains_key(&item.0) && item.1 + data[&item.0].g < tmp {
                tmp = item.1 + data[&item.0].g;
            }
        }
        data.get_mut(&s).unwrap().rhs = tmp;
    }
    for x in open.iter() {
        if x.state == s {
            println!("WARNING: Should remove s from open - needs: \
            https://github.com/rust-lang/rust/issues/66724; \
            This indicates there are cycles in the state space.");
        }
    }
    if data[&s].g != data[&s].rhs {
        open.push(util::HeapEntry::new_entry(
            s, key(&data[&s], ps.heuristic(&s, &start))));
    }
}

fn compute_path<PS: planner::ProblemSpace>(
    ps: &PS,
    start: PS::State,
    goal: PS::State,
    data: &mut collections::HashMap<PS::State, util::StateData>,
    open: &mut collections::BinaryHeap<util::HeapEntry<PS::State>>) {
    while (!open.is_empty()) &&
        ((open.peek().unwrap().keys < key(&data[&start],
                                          ps.heuristic(&start, &start))) ||
            (data[&start].rhs != data[&start].g)) {
        let s: util::HeapEntry<PS::State> = open.pop().unwrap();
        if data[&s.state].g > data[&s.state].rhs {
            data.get_mut(&s.state).unwrap().g = data[&s.state].rhs;
            for (s_dash, _) in ps.pred(&s.state) {
                update_state(ps, s_dash, start, goal, data, open);
            }
        } else {
            data.get_mut(&s.state).unwrap().g = f64::INFINITY;
            update_state(ps, s.state, start, goal, data, open);
            for (s_dash, _) in ps.pred(&s.state) {
                update_state(ps, s_dash, start, goal, data, open);
            }
        }
    }
}

///
/// Find a plan to get from start to goal, given a problem space.
///
pub fn solve<PS: planner::ProblemSpace>(
    ps: &PS, start: PS::State, goal: PS::State) -> Vec<PS::State> {
    let mut open: collections::BinaryHeap<util::HeapEntry<PS::State>> =
        collections::BinaryHeap::new();
    let mut data: collections::HashMap<PS::State, util::StateData> =
        collections::HashMap::new();

    data.insert(start, util::StateData { g: f64::INFINITY, rhs: f64::INFINITY });
    data.insert(goal, util::StateData { g: f64::INFINITY, rhs: 0.0 });
    open.push(util::HeapEntry::new_entry(
        goal, key(&data[&goal], ps.heuristic(&goal, &start))));

    compute_path(ps, start, goal, &mut data, &mut open);

    // TODO: check if useful to implement action ids to not only output the states but also the
    //   actions (aka. the path/edges -> the road taken).
    let mut res = Vec::new();
    let mut curr = start;
    while curr != goal {
        let mut min_cost = f64::INFINITY;
        let mut next_state = curr;
        for item in ps.succ(&curr) {
            if data.contains_key(&item.0) && data[&item.0].g < min_cost {
                min_cost = data[&item.0].g;
                next_state = item.0;
            }
        }
        res.push(next_state);
        curr = next_state;
    }
    // TODO: implement loop looking for obstacles & update accordingly...
    res
}

#[cfg(test)]
mod tests {
    use std::collections;
    use std::vec;

    use crate::dstar_lite;
    use crate::planner;
    use crate::util;

    struct SimpleGraph {}

    impl planner::ProblemSpace for SimpleGraph {
        type State = i32;
        type Iter = vec::IntoIter<(i32, f64)>;
        fn heuristic(&self, _: &Self::State, _: &Self::State) -> f64 {
            0.0
        }

        fn succ(&self, s: &Self::State) -> Self::Iter {
            match *s {
                0 => vec![(1, 1.0)].into_iter(),
                1 => vec![(2, 1.0), (3, 1.0)].into_iter(),
                2 => vec![(4, 1.0)].into_iter(),
                _ => vec![(4, 5.0)].into_iter()
            }
        }

        fn pred(&self, s: &Self::State) -> Self::Iter {
            match *s {
                1 => vec![(0, 1.0)].into_iter(),
                2 => vec![(1, 1.0)].into_iter(),
                3 => vec![(1, 1.0)].into_iter(),
                _ => vec![(2, 1.0), (3, 5.0)].into_iter()
            }
        }
    }

    // Test for success.

    #[test]
    fn test_key_success() {
        let s_d = util::StateData { g: 1.0, rhs: 1.0 };
        dstar_lite::key(&s_d, 1.0);
    }

    #[test]
    fn test_update_state_for_success() {
        let ps = SimpleGraph {};
        let start = 0;
        let goal = 4;

        let mut data = collections::HashMap::new();
        data.insert(start, util::StateData { g: f64::INFINITY, rhs: f64::INFINITY });
        data.insert(goal, util::StateData { g: f64::INFINITY, rhs: 0.0 });

        let mut open = collections::BinaryHeap::new();
        open.push(util::HeapEntry::new_entry(
            goal, dstar_lite::key(&data[&goal], 0.0)));

        let s = 1;
        dstar_lite::update_state(&ps, s, start, goal, &mut data, &mut open);
    }

    #[test]
    fn test_compute_path_for_success() {
        let ps = SimpleGraph {};
        let start = 0;
        let goal = 4;

        let mut data = collections::HashMap::new();
        data.insert(start, util::StateData { g: f64::INFINITY, rhs: f64::INFINITY });
        data.insert(goal, util::StateData { g: f64::INFINITY, rhs: 0.0 });

        let mut open = collections::BinaryHeap::new();
        open.push(util::HeapEntry::new_entry(
            goal, dstar_lite::key(&data[&goal], 0.0)));

        dstar_lite::compute_path(&ps, start, goal, &mut data, &mut open);
    }

    #[test]
    fn test_solve_for_success() {
        let ps = SimpleGraph {};
        dstar_lite::solve(&ps, 0, 4);
    }

    // Test for failure.

    // TODO: figure out what to do about this...

    // Test for sanity.

    #[test]
    fn test_key_sanity() {
        let sd_0 = util::StateData { g: 1.0, rhs: 1.0 };
        let sd_1 = util::StateData { g: 10.0, rhs: 1.0 };
        let sd_2 = util::StateData { g: 1.0, rhs: 10.0 };
        assert_eq!(dstar_lite::key(&sd_0, 1.0), (1.0, 1.0));
        assert_eq!(dstar_lite::key(&sd_1, 1.0), (2.0, 10.0));
        assert_eq!(dstar_lite::key(&sd_2, 1.0), (1.0, 10.0));
    }

    #[test]
    fn test_update_state_for_sanity() {
        let ps = SimpleGraph {};
        let start = 0;
        let goal = 4;

        let mut data = collections::HashMap::new();
        // data.insert(start, util::StateData { g: f64::INFINITY, rhs: f64::INFINITY });
        data.insert(goal, util::StateData { g: 1.0, rhs: 0.0 });
        let mut open = collections::BinaryHeap::new();

        // s not visited...
        let s = 1;
        dstar_lite::update_state(&ps, s, start, goal, &mut data, &mut open);
        assert_eq!(data.contains_key(&s), true);

        // s != goal
        let s = 2;
        dstar_lite::update_state(&ps, s, start, goal, &mut data, &mut open);
        assert_eq!(data[&s].rhs, 2.0);  // g(goal) == 1.0 + cost of 1.0

        // s should have been added to open queue...
        let mut contains = false;
        for item in open.iter() {
            if item.state == s {
                contains = true;
            }
        }
        assert_eq!(contains, true);
    }

    #[test]
    fn test_compute_path_for_sanity() {
        let ps = SimpleGraph {};
        let start = 0;
        let goal = 4;

        let mut data = collections::HashMap::new();
        data.insert(start, util::StateData { g: f64::INFINITY, rhs: f64::INFINITY });
        data.insert(goal, util::StateData { g: f64::INFINITY, rhs: 0.0 });

        let mut open = collections::BinaryHeap::new();
        open.push(util::HeapEntry::new_entry(
            goal, dstar_lite::key(&data[&goal], 0.0)));

        dstar_lite::compute_path(&ps, start, goal, &mut data, &mut open);

        assert_eq!(data[&0].g, 3.0);  // 3 steps to goal possible ...
        assert_eq!(data[&1].g, 2.0);  // 2 steps ...
        assert_eq!(data[&2].g, 1.0);
        assert_eq!(data[&3].g, f64::INFINITY);  // should not have expanded in A* search ...
        assert_eq!(data[&4].g, 0.0);
    }

    #[test]
    fn test_solve_for_sanity() {
        let ps = SimpleGraph {};
        assert_eq!(dstar_lite::solve(&ps, 0, 4), vec![1, 2, 4])
    }
}