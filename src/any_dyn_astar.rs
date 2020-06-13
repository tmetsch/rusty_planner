use std::collections;
use std::f64;

use crate::planner;
use crate::util;

/// Calculates the priority in the queue.
fn key(data: &util::StateData, h: f64, eps: f64) -> (f64, f64) {
    if data.g > data.rhs {
        (data.rhs + eps * h, data.rhs)
    } else {
        (data.g + h, data.g)
    }
}

/// Update the state's information.
fn update_state<PS: planner::ProblemSpace>(
    ps: &PS,
    s: PS::State,
    start: PS::State,
    goal: PS::State,
    data: &mut collections::HashMap<PS::State, util::StateData>,
    open: &mut collections::BinaryHeap<util::HeapEntry<PS::State>>,
    closed: &mut Vec<PS::State>,
    incons: &mut Vec<PS::State>,
    eps: f64,
) {
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
        if !closed.contains(&s) {
            open.push(util::HeapEntry::new_entry(
                s, key(&data[&s], ps.heuristic(&s, &start), eps)));
        } else {
            incons.push(s);
        }
    }
}

/// Compute or improve the path between start & goal.
fn compute_path<PS: planner::ProblemSpace>(
    ps: &PS,
    start: PS::State,
    goal: PS::State,
    data: &mut collections::HashMap<PS::State, util::StateData>,
    open: &mut collections::BinaryHeap<util::HeapEntry<PS::State>>,
    closed: &mut Vec<PS::State>,
    incons: &mut Vec<PS::State>,
    eps: f64) {
    while (!open.is_empty()) &&
        ((open.peek().unwrap().keys < key(&data[&start],
                                          ps.heuristic(&start, &start), eps)) ||
            (data[&start].rhs != data[&start].g)) {
        let s: util::HeapEntry<PS::State> = open.pop().unwrap();
        if data[&s.state].g > data[&s.state].rhs {
            data.get_mut(&s.state).unwrap().g = data[&s.state].rhs;
            closed.push(s.state);
            for (s_dash, _) in ps.pred(&s.state) {
                update_state(ps, s_dash, start, goal, data, open, closed, incons, eps);
            }
        } else {
            data.get_mut(&s.state).unwrap().g = f64::INFINITY;
            update_state(ps, s.state, start, goal, data, open, closed, incons, eps);
            for (s_dash, _) in ps.pred(&s.state) {
                update_state(ps, s_dash, start, goal, data, open, closed, incons, eps);
            }
        }
    }
}

///
/// Given an problem space will try to find most efficient path from start to goal.
///
pub fn solve<PS: planner::ProblemSpace>(
    ps: &PS,
    start: PS::State,
    goal: PS::State,
    callback: fn(Vec<PS::State>),
) {
    // initialize data - copy start & goal state into here...
    let mut node_data: collections::HashMap<PS::State, util::StateData> =
        collections::HashMap::new();
    node_data.insert(start, util::StateData { rhs: f64::INFINITY, g: f64::INFINITY });
    node_data.insert(goal, util::StateData { rhs: 0.0, g: f64::INFINITY });
    let eps = 2.0;

    let mut open: collections::BinaryHeap<util::HeapEntry<PS::State>> =
        collections::BinaryHeap::new();
    let mut closed: Vec<PS::State> = Vec::new();
    let mut incons: Vec<PS::State> = Vec::new();
    let node_0: util::HeapEntry<PS::State> = util::HeapEntry::new_entry(
        goal, key(&node_data[&goal], ps.heuristic(&goal, &start), eps));
    open.push(node_0);

    // compute path
    compute_path(ps, start, goal, &mut node_data, &mut open, &mut closed, &mut incons, eps);

    // publish sub-optimal result.
    publish_plan(ps, start, goal, node_data, callback);

    // TODO: implement the forever loop...
}

/// Uses callback to publish the current solution.
fn publish_plan<PS: planner::ProblemSpace>(ps: &PS,
                                           start: PS::State,
                                           goal: PS::State,
                                           data: collections::HashMap<PS::State, util::StateData>,
                                           callback: fn(Vec<PS::State>)) {
    let mut plan = Vec::new();
    let mut done = false;
    let mut curr = start;
    while !done {
        let mut min_cost = f64::INFINITY;
        let mut next_state = curr;
        for successor in ps.succ(&curr) {
            if data.contains_key(&successor.0) &&
                data[&successor.0].g + successor.1 <= min_cost {
                min_cost = data[&successor.0].g + successor.1;
                next_state = successor.0;
            }
        }
        curr = next_state;
        plan.push(curr);
        // By checking second part - we would also publish incomplete plans...
        if curr == goal || data[&curr].rhs == f64::INFINITY {
            done = true;
        }
    }

    callback(plan);
}

#[cfg(test)]
mod tests {
    use std::collections;
    use std::vec;

    use crate::any_dyn_astar;
    use crate::planner;
    use crate::util;

    struct Example {}

    impl planner::ProblemSpace for Example {
        type State = i32;
        type Iter = vec::IntoIter<(i32, f64)>;
        fn heuristic(&self, _: &Self::State, _: &Self::State) -> f64 {
            1.0
        }
        fn succ(&self, _: &Self::State) -> Self::Iter { vec![(1, 1.0)].into_iter() }
        fn pred(&self, _: &Self::State) -> Self::Iter { vec![(0, 1.0)].into_iter() }
    }

    fn callback(_: Vec<i32>) {}

    // Test for success.

    #[test]
    fn test_key_for_success() {
        let s_0 = util::StateData { rhs: 1.0, g: 2.0 };
        any_dyn_astar::key(&s_0, 1.0, 1.0);
    }

    #[test]
    fn test_update_state_for_success() {
        let ps = Example {};
        let start = 0;
        let goal = 1;
        let s = 0;
        let mut data = collections::HashMap::new();
        let mut open: collections::BinaryHeap<util::HeapEntry<i32>> =
            collections::BinaryHeap::new();
        let mut closed = Vec::new();
        let mut incons = Vec::new();
        any_dyn_astar::update_state(
            &ps, s, start, goal, &mut data, &mut open, &mut closed, &mut incons, 1.0);
    }

    #[test]
    fn test_compute_path_for_success() {
        let ps = Example {};
        let start = 0;
        let goal = 1;
        let mut data = collections::HashMap::new();
        let mut open: collections::BinaryHeap<util::HeapEntry<i32>> =
            collections::BinaryHeap::new();
        let mut closed = Vec::new();
        let mut incons = Vec::new();
        any_dyn_astar::compute_path(
            &ps, start, goal, &mut data, &mut open, &mut closed, &mut incons, 1.0);
    }

    #[test]
    fn test_publish_plan_for_success() {
        let ps = Example {};
        let start = 0;
        let goal = 1;
        let mut data = collections::HashMap::new();
        data.insert(goal, util::StateData { g: 1.0, rhs: 1.0 });
        any_dyn_astar::publish_plan(&ps, start, goal, data, callback);
    }

    // Test for failure.

    // TODO: figure this one out!

    // Test for sanity.

    #[test]
    fn test_key_for_sanity() {
        let s_0 = util::StateData { rhs: 1.0, g: 2.0 };
        let s_1 = util::StateData { rhs: 2.0, g: 2.0 };
        let eps = 1.0;
        let h = 1.0;
        assert_eq!(any_dyn_astar::key(&s_0, h, eps), (2., 1.));  // 1 + 1 * 1, 1
        assert_eq!(any_dyn_astar::key(&s_1, h, eps), (3., 2.));  // 2 + 1, 2
    }

    #[test]
    fn test_update_state_for_sanity() {
        let ps = Example {};
        let start = 0;
        let goal = 1;
        let mut data = collections::HashMap::new();
        data.insert(start, util::StateData { g: 1.0, rhs: 1.0 });
        data.insert(goal, util::StateData { g: 1.0, rhs: 1.0 });
        let mut open: collections::BinaryHeap<util::HeapEntry<i32>> =
            collections::BinaryHeap::new();
        let mut closed = Vec::new();
        let mut incons = Vec::new();

        // unknown state - add to data...
        let s = 2;
        any_dyn_astar::update_state(
            &ps, s, start, goal, &mut data, &mut open, &mut closed, &mut incons, 1.0);
        assert_eq!(data[&2].g, f64::INFINITY);
        assert_eq!(data[&2].rhs, 2.0);  // cost 1.0 + g(succ(s)) 1.0 == 2.0

        // state != goal
        let s = 0;
        any_dyn_astar::update_state(
            &ps, s, start, goal, &mut data, &mut open, &mut closed, &mut incons, 1.0);
        assert_eq!(data[&0].rhs, 2.0);

        // g != rhs
        let s = 3;
        data.insert(s, util::StateData { g: 1.0, rhs: 10.0 });
        any_dyn_astar::update_state(
            &ps, s, start, goal, &mut data, &mut open, &mut closed, &mut incons, 1.0);
        let mut contains = false;
        for item in open.iter() {
            if item.state == s {
                contains = true;
            }
        }
        assert_eq!(contains, true);  // should have been added to list...

        closed.push(s);
        any_dyn_astar::update_state(
            &ps, s, start, goal, &mut data, &mut open, &mut closed, &mut incons, 1.0);
        assert_eq!(incons.contains(&s), true);
    }

    #[test]
    fn test_compute_path_for_sanity() {
        let ps = Example {};
        let start = 0;
        let goal = 1;
        let mut data = collections::HashMap::new();
        let mut open: collections::BinaryHeap<util::HeapEntry<i32>> =
            collections::BinaryHeap::new();
        let mut closed = Vec::new();
        let mut incons = Vec::new();

        data.insert(start, util::StateData { rhs: f64::INFINITY, g: f64::INFINITY });
        data.insert(goal, util::StateData { rhs: 0.0, g: f64::INFINITY });

        let node_0 = util::HeapEntry::new_entry(
            goal, any_dyn_astar::key(&data[&1], 1.0, 1.0));
        open.push(node_0);

        any_dyn_astar::compute_path(
            &ps, start, goal, &mut data, &mut open, &mut closed, &mut incons, 1.0);
        assert_eq!(open.len(), 0);
        assert_eq!(data[&0].g, 1.);  // 1 step to get to goal...
        assert_eq!(data[&0].rhs, 1.);
        assert_eq!(data[&1].g, 0.);  // it's the goal :-)
        assert_eq!(data[&1].rhs, 0.);
    }
}
