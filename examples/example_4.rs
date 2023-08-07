#[cfg(feature = "multi_agent")]
use std::env;
#[cfg(feature = "multi_agent")]
use std::thread;
#[cfg(feature = "multi_agent")]
use std::time;
use std::vec;

#[cfg(feature = "multi_agent")]
use rusty_agent::agent;
#[cfg(feature = "multi_agent")]
use rusty_planner::mad_astar;
use rusty_planner::planner;

struct Picker {
    pick_id: i32,
}

///
/// Let's assume we've two picker robots - only together they can move an object from one location
/// to another.
///
///    |---|   |---|
///    |   |   | G |
///    |---|---|---|
/// P0 |   |   |   | P1
///    |---|---|---|
///    | S |   |   |
///    |---|   |---|
///
/// Picker with ID 0 can move package from coordinates (0, 0) to (1, 1) (which is the handover
/// location), and picker with ID 1 can move the package from (1, 1) to the goal of (2, 2).
///
impl planner::ProblemSpace for Picker {
    type State = (i32, i32);
    type Iter = vec::IntoIter<(Self::State, f64)>;

    fn heuristic(&self, _: &Self::State, _: &Self::State) -> f64 {
        0.0
    }

    fn succ(&self, state: &Self::State) -> Self::Iter {
        match (*state, self.pick_id) {
            // Picker 0 can move diagonal...
            ((0, 0), 0) => vec![((0, 1), 0.5), ((1, 1), 0.7)].into_iter(),
            ((0, 1), 0) => vec![((0, 0), 0.5), ((0, 2), 0.5), ((1, 1), 0.5)].into_iter(),
            ((0, 2), 0) => vec![((0, 1), 0.5), ((1, 1), 0.7)].into_iter(),
            ((1, 1), 0) => vec![((0, 0), 0.7), ((0, 1), 0.5), ((0, 2), 0.7)].into_iter(),
            // ...picker 1 is a bit more dumb.
            ((1, 1), 1) => vec![((2, 1), 0.5)].into_iter(),
            ((2, 0), 1) => vec![((2, 1), 0.5)].into_iter(),
            ((2, 1), 1) => vec![((2, 0), 0.5), ((2, 2), 0.5), ((1, 1), 0.5)].into_iter(),
            ((2, 2), 1) => vec![((2, 1), 0.5)].into_iter(),
            _ => vec![].into_iter(),
        }
    }
    fn pred(&self, state: &Self::State) -> Self::Iter {
        self.succ(state)
    }
}

impl planner::SharedStates for Picker {
    fn is_public(&self, state: &Self::State) -> bool {
        matches!(state, (1, 1))
    }

    fn serialize(&self, msg_type: u8, state: &Self::State, para: Vec<f64>) -> String {
        let mut string_list: Vec<String> = vec![
            msg_type.to_string(),
            state.0.to_string(),
            state.1.to_string(),
        ];
        for item in para {
            string_list.push(item.to_string());
        }
        string_list.join(";")
    }

    fn deserialize(&self, msg: String) -> (u8, Self::State, Vec<f64>) {
        let split = msg.split(';');
        let data = split.collect::<Vec<&str>>();

        let msg_type = data[0].parse::<u8>().unwrap();
        let state = (
            data[1].parse::<i32>().unwrap(),
            data[2].parse::<i32>().unwrap(),
        );
        let mut para: Vec<f64> = vec![];
        if data.len() > 3 {
            para.insert(0, data[3].parse::<f64>().unwrap());
            para.insert(1, data[4].parse::<f64>().unwrap());
        }

        (msg_type, state, para)
    }
}

#[cfg(feature = "multi_agent")]
/// Starts an agent of a multi-agent system.
fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() <= 1 {
        panic!("Usage: cargo run --example example_4 <picker id (0|1)>");
    }

    let picker_id = &args[1];
    let ep = "tcp://127.0.0.1:800".to_owned() + picker_id;
    let agent = agent::ZeroAgent::builder(ep.clone()).build();

    // If I'm not the the first agent - connect myself to the one started before me...
    if picker_id
        .parse::<u32>()
        .expect("Provided Id is not an u32...")
        > 0
    {
        let ngbh_id = picker_id.parse::<u32>().unwrap() - 1;
        let other_ep = "tcp://127.0.0.1:800".to_owned() + &ngbh_id.to_string();
        agent.add_peer(other_ep)
    }

    // Setup...
    let ps = Picker {
        pick_id: picker_id.parse().unwrap(),
    };
    let ths = agent.activate();

    // Assure we have a partner ready...
    let mut i = 0;
    let mut ready = false;
    while i < 10 {
        if agent.get_n_peers() > 1 {
            ready = true;
        }
        thread::sleep(time::Duration::from_millis(250));
        i += 1;
    }
    if !ready {
        panic!("Could not find peers!");
    }

    // Solve this mystery.
    let start = (0, 0);
    let goal = (2, 2);
    let res = mad_astar::solve(&agent, &ps, start, goal);
    agent.send_msg(&ep, &agent::Msg::Kill());
    ths.0.join().unwrap();
    ths.1.join().unwrap();

    println!("My part of moving from {:?} to {:?} is:", start, goal);
    for step in res {
        print!("{:?} -> ", step);
    }
    println!("\ndone...");
}

#[cfg(not(feature = "multi_agent"))]
fn main() {
    println!("Make sure to run this example with multi_agent feature enabled...");
}
