use std::sync;
use std::thread;
use std::time;
use std::vec;

// TODO: look into caching connections, etc.
// TODO: look into agents advertising capabilities - OCCI style of course :-)

/// Timeout for pinging peers in sec.
const TIMEOUT: u64 = 2;
/// Time to grand other agents to respond in ms.
const WAIT: u64 = 100;

/// Type of messages that can be passed between agents.
pub enum Msg {
    Ping(String),
    Message(String),
    Kill(),
}

impl Msg {
    /// Convert a message - ensures format as the listener expects it.
    pub fn to_msg(&self) -> String {
        match &self {
            Msg::Ping(content) => String::from("P@") + content,
            Msg::Message(content) => String::from("M@") + content,
            Msg::Kill() => String::from("K@0")
        }
    }
}

///
/// An agent is defined by its endpoint and set of neighbours.
///
pub struct Agent {
    ep: String,
    peers: sync::Arc<sync::Mutex<vec::Vec<String>>>,
    msgs: sync::Arc<sync::Mutex<vec::Vec<String>>>,
    ctxt: zmq::Context,
}

///
/// Very simple implementation of an agent in a multi-agent system. Agents can join and leave the
/// system on the fly. No centralized component is needed - probably also not the nicest solution
/// available, but does the trick for now.
///
/// Using ZeroMQ for comms - using REP/REQ sockets atm; Could be done more elegant with other
/// socket type - but works for now - KISS :-)
///
impl Agent {
    /// Creates a new agent.
    // TODO: check: Result<Agent, Error>
    pub fn new(ep: String) -> Agent {
        let ngbhs: sync::Arc<sync::Mutex<Vec<String>>> =
            sync::Arc::new(sync::Mutex::new(vec![ep.clone()]));
        let msgs: sync::Arc<sync::Mutex<Vec<String>>> =
            sync::Arc::new(sync::Mutex::new(vec![]));
        let context: zmq::Context = zmq::Context::new();
        Agent { ep, peers: ngbhs, ctxt: context, msgs }
    }

    /// add a peer to the multi-agent system.
    pub fn add_peer(&self, ep: String) {
        let rcp: sync::Arc<sync::Mutex<Vec<String>>> = sync::Arc::clone(&self.peers);
        let mut peers: sync::MutexGuard<Vec<String>> = rcp.lock().unwrap();
        if (!peers.contains(&ep)) && ep != self.ep {
            peers.push(ep);
        }
        drop(peers);
    }

    /// Send a message to an agent's peers.
    pub fn send_msg(&self, msg: Msg) {
        let rcp: sync::Arc<sync::Mutex<Vec<String>>> = sync::Arc::clone(&self.peers);
        let peers: sync::MutexGuard<Vec<String>> = rcp.lock().unwrap();
        for peer in peers.iter() {
            if peer != &self.ep {
                let client = &self.ctxt.socket(zmq::REQ).unwrap();
                client.connect(&peer).expect("Could not connect to peer");
                client.send(msg.to_msg().as_str(), 0).unwrap();
                client.recv_msg(0).unwrap();  // Wait for ack...
            }
        }
        drop(peers);
    }

    /// Activate the agents - will start listener and mgmt. threads.
    pub fn activate(&self) -> (thread::JoinHandle<()>, thread::JoinHandle<()>) {
        // The listener threads, watches for incoming messages.
        let rcp_0: sync::Arc<sync::Mutex<Vec<String>>> = sync::Arc::clone(&self.peers);
        let msgs: sync::Arc<sync::Mutex<Vec<String>>> = sync::Arc::clone(&self.msgs);
        let ep_0: String = self.ep.clone();
        let ctxt_0: zmq::Context = self.ctxt.clone();
        let list_th = thread::spawn(move || {
            listen(ctxt_0, ep_0, rcp_0, msgs);
        });

        // Ping thread - assures reasonably consistency.
        let rcp_1: sync::Arc<sync::Mutex<Vec<String>>> = sync::Arc::clone(&self.peers);
        let ep_1: String = self.ep.clone();
        let ctxt_1: zmq::Context = self.ctxt.clone();
        let ping_th = thread::spawn(move || {
            ping(ctxt_1, ep_1, rcp_1);
        });
        (list_th, ping_th)
    }
}

/// Listen to incoming messages and act accordingly.
fn listen(ctxt: zmq::Context,
          ep: String,
          rcp: sync::Arc<sync::Mutex<Vec<String>>>,
          msg_rcp: sync::Arc<sync::Mutex<Vec<String>>>) {
    let mut done: bool = false;
    let list: zmq::Socket = ctxt.socket(zmq::REP).unwrap();
    list.bind(&ep).expect("Could not bind...");

    while !done {
        let msg: zmq::Message = list.recv_msg(0).unwrap();
        let tmp: String = msg.as_str().unwrap().to_string();
        list.send("0", 0).unwrap();

        let split = tmp.split('@');
        let data = split.collect::<Vec<&str>>();

        if data[0] == "P" {
            let mut peers: sync::MutexGuard<Vec<String>> = rcp.lock().unwrap();
            for peer in data[1].to_string().split(',') {
                if !peers.contains(&peer.to_string()) {
                    peers.push(peer.to_string());
                }
            }
            drop(peers);
        } else if data[0] == "M" {
            let mut msgs: sync::MutexGuard<Vec<String>> = msg_rcp.lock().unwrap();
            msgs.push(String::from(data[1]));
            drop(msgs);
        } else if data[0] == "K" {
            let mut peers: sync::MutexGuard<Vec<String>> = rcp.lock().unwrap();
            peers.clear();
            drop(peers);
            done = true;
        }
    }
}

///
/// Will on a given timout try to ping the host it knows and if needed remove peers from the list
/// of known neighbours.
///
/// Could be optimized by only sending delta in data between last msg and new one.
///
fn ping(ctxt: zmq::Context, my_ep: String, rcp: sync::Arc<sync::Mutex<Vec<String>>>) {
    let mut done: bool = false;
    while !done {
        let mut peers: sync::MutexGuard<Vec<String>> = rcp.lock().unwrap();
        let joined = peers.join(",");
        let msg = Msg::Ping(joined);
        let mut dead_peers: Vec<String> = vec::Vec::new();
        for peer in peers.iter() {
            if peer != &my_ep {
                // Just to be safe (old socket could be in weird state) I'm getting a new socket...
                let client = ctxt.socket(zmq::REQ).unwrap();
                client.set_connect_timeout(2).unwrap();
                // TODO: would be great to set: ZMQ_REQ_CORRELATE; not support atm.
                client.connect(&peer).expect("Could not connect to peer");
                client.send(msg.to_msg().as_str(), 0).unwrap();
                thread::sleep(time::Duration::from_millis(WAIT));
                if client.recv_msg(zmq::DONTWAIT).is_err() {
                    dead_peers.push(peer.clone());
                }
                client.disconnect(&peer).unwrap();
            }
        }
        for dead in dead_peers.iter() {
            peers.retain(|x: &String| x != dead);
        }

        if peers.is_empty() {
            done = true;
        }

        drop(peers);
        thread::sleep(time::Duration::from_secs(TIMEOUT));
    }
}

#[cfg(test)]
mod tests {
    use std::thread;
    use std::time;

    use crate::agent;

    fn send_kill(ep: &str) {
        let ctxt = zmq::Context::new();
        let client = ctxt.socket(zmq::REQ).unwrap();
        client.connect(&ep).expect("Could not connect to peer");
        client.send(agent::Msg::Kill().to_msg().as_str(), 0).unwrap();
        client.recv_msg(0);
        client.disconnect(&ep).unwrap();
    }

    // Test for success.

    #[test]
    fn test_new_for_success() {
        agent::Agent::new("inproc://#0".to_string());
    }

    #[test]
    fn test_add_peer_for_success() {
        let a_0 = agent::Agent::new("inproc://#1".to_string());
        a_0.add_peer("inproc://#1".to_string());
    }

    #[test]
    fn test_send_msg_for_success() {
        let a_0 = agent::Agent::new("tcp://127.0.0.1:8787".to_string());
        let th0 = a_0.activate();
        let a_1 = agent::Agent::new("tcp://127.0.0.1:8989".to_string());
        let th1 = a_1.activate();
        a_1.add_peer("tcp://127.0.0.1:8787".to_string());

        thread::sleep(time::Duration::from_millis(200));
        a_1.send_msg(agent::Msg::Message(String::from("hello")));

        send_kill("tcp://127.0.0.1:8787");
        send_kill("tcp://127.0.0.1:8989");
        th0.0.join().unwrap();
        th0.1.join().unwrap();
        th1.0.join().unwrap();
        th1.1.join().unwrap();
    }

    #[test]
    fn test_activate_for_success() {
        let a_0 = agent::Agent::new("tcp://127.0.0.1:1234".to_string());
        let ths = a_0.activate();
        thread::sleep(time::Duration::from_millis(100));
        send_kill("tcp://127.0.0.1:1234");
        ths.0.join().unwrap();
        ths.1.join().unwrap();
    }

    // Test for failure.

    // TODO: figure this out...

    // Test for sanity.

    #[test]
    fn test_send_msg_for_sanity() {
        let a_0 = agent::Agent::new("tcp://127.0.0.1:5000".to_string());
        a_0.activate();
        let a_1 = agent::Agent::new("tcp://127.0.0.1:5001".to_string());
        a_1.add_peer(String::from("tcp://127.0.0.1:5000"));
        a_1.activate();

        thread::sleep(time::Duration::from_millis(200));
        a_0.send_msg(agent::Msg::Message(String::from("Hello")));

        // a_0 was sender so msg list is empty...
        assert!(a_0.msgs.lock().unwrap().to_vec().is_empty());
        // a_1 should have received a hello...
        assert_eq!(a_1.msgs.lock().unwrap().to_vec(), vec!["Hello"]);

        send_kill("tcp://127.0.0.1:5000");
        send_kill("tcp://127.0.0.1:5001");
    }

    #[test]
    fn test_activate_for_sanity() {
        let a_0 = agent::Agent::new("tcp://127.0.0.1:5002".to_string());
        let a_1 = agent::Agent::new("tcp://127.0.0.1:5003".to_string());
        a_1.add_peer("inproc://#2".to_string());

        a_0.activate();
        a_1.activate();

        thread::sleep(time::Duration::from_millis(100));
        send_kill("tcp://127.0.0.1:5003");
        // When a_1 is gone, a_0 should only know itself...
        assert_eq!(a_0.peers.lock().unwrap().to_vec(), vec!["tcp://127.0.0.1:5002"]);
        send_kill("tcp://127.0.0.1:5002");
    }
}