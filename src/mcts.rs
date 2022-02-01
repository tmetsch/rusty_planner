use std::collections;

use crate::planner;

/// Expand the given state.
fn expand<PS: planner::ProblemSpace>(
    ps: &PS,
    v: PS::State,
    children: &mut collections::HashMap<PS::State, Vec<PS::State>>,
) -> Option<PS::State> {
    let mut res: PS::State = v;
    for item in ps.succ(&v) {
        if !children.contains_key(&v) || !children[&v].contains(&item.0) {
            children.entry(v).or_insert_with(Vec::new).push(item.0);
            res = item.0;
            break;
        }
    }
    if res != v {
        Some(res)
    } else {
        None
    }
}

/// Determine the best possible next step.
fn best_child<'a, PS: planner::ProblemSpace + planner::Anytime>(
    _: &'a PS,
    v: PS::State,
    children: &mut collections::HashMap<PS::State, Vec<PS::State>>,
    n_vals: &collections::HashMap<PS::State, u64>,
    q_vals: &collections::HashMap<PS::State, f64>,
    c_val: f64,
) -> Option<PS::State> {
    let mut max_val: f64 = 0.0;
    let mut res = v;
    for child in children.get(&v).unwrap() {
        let tmp: f64 = if c_val > 0.0 {
            (q_vals[child] / n_vals[child] as f64)
                + c_val * (((2.0 * (n_vals[&v] as f64).ln()) / n_vals[child] as f64).sqrt())
        } else {
            q_vals[child] / n_vals[child] as f64
        };
        if tmp >= max_val {
            max_val = tmp;
            res = *child;
        }
    }
    if res != v {
        Some(res)
    } else {
        None
    }
}

/// Select of expand a state.
fn tree_policy<'a, PS: planner::ProblemSpace + planner::Anytime>(
    ps: &'a PS,
    state: PS::State,
    children: &mut collections::HashMap<PS::State, Vec<PS::State>>,
    n_vals: &collections::HashMap<PS::State, u64>,
    q_vals: &collections::HashMap<PS::State, f64>,
) -> PS::State {
    let mut v = state;
    while ps.succ(&v).count() != 0 {
        if !children.contains_key(&v) || ps.succ(&v).count() != (children.get(&v).unwrap()).len() {
            v = expand(ps, v, children).unwrap();
            break;
        } else {
            v = best_child(ps, v, children, n_vals, q_vals, 1.0).unwrap();
        }
    }
    v
}

/// Simulate what would happen if you play from this state to the end.
// TODO: check if to make this part of the trait - would allow for multi-player games etc.
fn default_policy<PS: planner::ProblemSpace + planner::Anytime>(
    ps: &PS,
    v: PS::State,
    parents: &mut collections::HashMap<PS::State, PS::State>,
) -> f64 {
    let mut s = v;
    let mut reward = 0.0;
    // This policy currently takes a greedy approach - random might be better in certain cases e.g.
    // if initially the cost of going one way is cheaper then a detour, but the way is blocked down
    // the road.
    while ps.succ(&s).count() != 0 {
        let mut min_val = f64::INFINITY;
        let mut tmp = s;
        for next_child in ps.succ(&s) {
            if next_child.1 <= min_val {
                min_val = next_child.1;
                tmp = next_child.0;
            }
        }
        parents.insert(tmp, s);
        s = tmp;
        reward += min_val;
    }

    // TODO: Check this - currently solution with lowest cost to get there is best.
    reward = 1.0 + (1.0 / reward);
    reward
}

/// Backpropagate the reward up the tree.
fn backup<PS: planner::ProblemSpace + planner::Anytime>(
    _: &PS,
    v: PS::State,
    delta: f64,
    n_vals: &mut collections::HashMap<PS::State, u64>,
    q_vals: &mut collections::HashMap<PS::State, f64>,
    parents: &collections::HashMap<PS::State, PS::State>,
) {
    let mut node = v;
    loop {
        *n_vals.entry(node).or_default() += 1;
        *q_vals.entry(node).or_default() += delta;
        if parents.contains_key(&node) {
            node = *parents.get(&node).unwrap();
        } else {
            break;
        }
    }
}

///
/// Given an problem space will try to figure our what the best next step/action is.
///
// TODO: Parallelize this.
pub fn solve<PS: planner::ProblemSpace + planner::Anytime>(
    ps: &mut PS,
    start: PS::State,
    goal: PS::State,
    iterations: u16,
) -> PS::State {
    let mut n_vals: collections::HashMap<PS::State, u64> = collections::HashMap::new();
    let mut q_vals: collections::HashMap<PS::State, f64> = collections::HashMap::new();
    let mut children: collections::HashMap<PS::State, Vec<PS::State>> = collections::HashMap::new();

    let mut curr = start;

    while curr != goal {
        let mut i: u16 = 0;
        while i < iterations {
            let mut parents: collections::HashMap<PS::State, PS::State> =
                collections::HashMap::new();
            let v_i = tree_policy(ps, curr, &mut children, &n_vals, &q_vals);
            parents.insert(v_i, curr);
            let delta = default_policy(ps, v_i, &mut parents);
            backup(ps, v_i, delta, &mut n_vals, &mut q_vals, &parents);
            i += 1;
        }
        curr = best_child(ps, curr, &mut children, &n_vals, &q_vals, 0.0).unwrap();
        ps.callback(&curr);
    }
    curr
}

#[cfg(test)]
mod tests {
    use std::collections;
    use std::vec;

    use crate::mcts;
    use crate::planner;
    use crate::planner::ProblemSpace;

    struct StateGraph {}

    impl planner::ProblemSpace for StateGraph {
        type State = i32;
        type Iter = vec::IntoIter<(Self::State, f64)>;

        fn heuristic(&self, _: &Self::State, _: &Self::State) -> f64 {
            panic!("Whoops...");
        }

        fn succ(&self, s_0: &Self::State) -> Self::Iter {
            match *s_0 {
                1 => vec![(2, 0.8), (3, 1.0)].into_iter(),
                2 => vec![(4, 1.0)].into_iter(),
                3 => vec![(4, 0.5), (5, 1.0)].into_iter(),
                4 => vec![(5, 0.8)].into_iter(),
                5 => vec![(6, 1.0)].into_iter(),
                _ => vec![].into_iter(),
            }
        }

        fn pred(&self, _: &Self::State) -> Self::Iter {
            unimplemented!()
        }
    }

    impl planner::Anytime for StateGraph {
        fn callback(&mut self, _: &Self::State) {}
    }

    // Test for success.

    #[test]
    fn test_expand_for_success() {
        let ps = StateGraph {};
        let mut children: collections::HashMap<i32, Vec<i32>> = collections::HashMap::new();

        mcts::expand(&ps, 1, &mut children);
    }

    #[test]
    fn test_best_child_for_success() {
        let ps = StateGraph {};

        let mut visits: collections::HashMap<i32, u64> = collections::HashMap::new();
        let mut q_vals: collections::HashMap<i32, f64> = collections::HashMap::new();
        let mut children: collections::HashMap<i32, Vec<i32>> = collections::HashMap::new();

        children.insert(1, vec![2, 3]);
        visits.insert(1, 1);
        visits.insert(2, 1);
        visits.insert(3, 2);
        q_vals.insert(2, 0.1);
        q_vals.insert(3, 0.1);

        mcts::best_child(&ps, 1, &mut children, &visits, &q_vals, 1.0);
    }

    #[test]
    fn test_tree_policy_for_success() {
        let ps = StateGraph {};

        let visits: collections::HashMap<i32, u64> = collections::HashMap::new();
        let q_vals: collections::HashMap<i32, f64> = collections::HashMap::new();
        let mut children: collections::HashMap<i32, Vec<i32>> = collections::HashMap::new();

        mcts::tree_policy(&ps, 1, &mut children, &visits, &q_vals);
    }

    #[test]
    fn test_default_policy_for_success() {
        let ps = StateGraph {};
        let mut parents: collections::HashMap<
            <StateGraph as ProblemSpace>::State,
            <StateGraph as ProblemSpace>::State,
        > = collections::HashMap::new();

        mcts::default_policy(&ps, 1, &mut parents);
    }

    #[test]
    fn test_backup_for_success() {
        let ps = StateGraph {};

        let mut visits: collections::HashMap<i32, u64> = collections::HashMap::new();
        let mut q_vals: collections::HashMap<i32, f64> = collections::HashMap::new();
        let mut parents: collections::HashMap<
            <StateGraph as ProblemSpace>::State,
            <StateGraph as ProblemSpace>::State,
        > = collections::HashMap::new();

        mcts::backup(&ps, 1, 1.0, &mut visits, &mut q_vals, &mut parents);
    }

    #[test]
    fn test_solve_for_success() {
        let mut ps = StateGraph {};
        mcts::solve(&mut ps, 1, 6, 3);
    }

    // Test for failure.

    #[test]
    #[should_panic(expected = "Whoops")]
    fn test_heuristic_for_failure() {
        let ps = StateGraph {};
        // just to get line coverage to 100% :-)
        ps.heuristic(&0, &1);
    }

    #[test]
    #[should_panic(expected = "not implemented")]
    fn test_pred_for_failure() {
        let ps = StateGraph {};
        // just to get line coverage to 100% :-)
        ps.pred(&0);
    }

    // Test for sanity.

    #[test]
    fn test_expand_for_sanity() {
        let ps = StateGraph {};
        let mut children: collections::HashMap<i32, Vec<i32>> = collections::HashMap::new();

        // should add 3 to the list of children of 1.
        children.insert(1, vec![2]);
        let res = mcts::expand(&ps, 1, &mut children);
        assert_eq!(children[&1], vec![2, 3]);
        assert_eq!(res.unwrap(), 3);

        // now all children have been found --> None.
        let res = mcts::expand(&ps, 1, &mut children);
        assert_eq!(res, None);
    }

    #[test]
    fn test_best_child_for_sanity() {
        let ps = StateGraph {};

        let mut visits: collections::HashMap<i32, u64> = collections::HashMap::new();
        let mut q_vals: collections::HashMap<i32, f64> = collections::HashMap::new();
        let mut children: collections::HashMap<i32, Vec<i32>> = collections::HashMap::new();

        children.insert(1, vec![2, 3]);
        children.insert(6, vec![]);
        visits.insert(1, 1);
        visits.insert(2, 2);
        visits.insert(3, 3);
        q_vals.insert(2, 0.75);
        q_vals.insert(3, 2.5);

        // (Exploration) 3 has a high q_val --> best child.
        let res = mcts::best_child(&ps, 1, &mut children, &visits, &q_vals, 1.0);
        assert_eq!(res.unwrap(), 3);

        // (next step) c val --> 0.0
        let res = mcts::best_child(&ps, 1, &mut children, &visits, &q_vals, 0.0);
        assert_eq!(res.unwrap(), 3);

        // no child states.
        let res = mcts::best_child(&ps, 6, &mut children, &visits, &q_vals, 0.0);
        assert_eq!(res, None);
    }

    #[test]
    fn test_tree_policy_for_sanity() {
        let ps = StateGraph {};

        let mut visits: collections::HashMap<i32, u64> = collections::HashMap::new();
        let mut q_vals: collections::HashMap<i32, f64> = collections::HashMap::new();
        let mut children: collections::HashMap<i32, Vec<i32>> = collections::HashMap::new();

        // no children - return itself.
        let res = mcts::tree_policy(&ps, 6, &mut children, &visits, &q_vals);
        assert_eq!(res, 6);

        // in case we expand - need to look into 3...walk all the way to the end...
        children.insert(1, vec![2]);
        let res2 = mcts::tree_policy(&ps, 1, &mut children, &visits, &q_vals);
        assert_eq!(res2, 3);

        // in case we try to find best child
        children.insert(3, vec![4, 5]);
        visits.insert(3, 1);
        visits.insert(4, 2);
        q_vals.insert(4, 1.0);
        visits.insert(5, 2);
        q_vals.insert(5, 0.8);
        let res3 = mcts::tree_policy(&ps, 3, &mut children, &visits, &q_vals);
        assert_eq!(res3, 5);
    }

    #[test]
    fn test_default_policy_for_sanity() {
        let ps = StateGraph {};
        let mut parents: collections::HashMap<
            <StateGraph as ProblemSpace>::State,
            <StateGraph as ProblemSpace>::State,
        > = collections::HashMap::new();

        let res = mcts::default_policy(&ps, 1, &mut parents);
        // cost of path (1->2->4->5->6) = 3.6 --> 1 + 1/3.6 = 1.28 --> round() --> 1.0
        assert_eq!(res.round(), 1.0);
    }

    #[test]
    fn test_backup_for_sanity() {
        let ps = StateGraph {};

        let mut visits: collections::HashMap<i32, u64> = collections::HashMap::new();
        let mut q_vals: collections::HashMap<i32, f64> = collections::HashMap::new();
        let mut parents: collections::HashMap<
            <StateGraph as ProblemSpace>::State,
            <StateGraph as ProblemSpace>::State,
        > = collections::HashMap::new();

        parents.insert(4, 3);
        parents.insert(3, 2);
        parents.insert(2, 1);
        mcts::backup(&ps, 4, 1.2, &mut visits, &mut q_vals, &mut parents);

        assert_eq!(visits[&3], 1);
        assert_eq!(visits[&1], 1);
        assert_eq!(q_vals[&3], 1.2);
        assert_eq!(q_vals[&1], 1.2);
    }

    #[test]
    fn test_solve_for_sanity() {
        let mut ps = StateGraph {};
        let res = mcts::solve(&mut ps, 1, 6, 3);
        assert_eq!(res, 6);
        let res = mcts::solve(&mut ps, 6, 6, 3);
        assert_eq!(res, 6);
    }
}
