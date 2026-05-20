use drone_sim::runner::SimFrame;

#[derive(Debug)]
pub struct AssertionResult {
    pub metric: String,
    pub value: f64,
    pub max: f64,
    pub passed: bool,
}

pub struct ScenarioReport {
    pub name: String,
    pub passed: bool,
    pub duration_s: f64,
    pub frame_count: usize,
    pub assertions: Vec<AssertionResult>,
    pub frames: Vec<SimFrame>,
}

impl ScenarioReport {
    pub fn print(&self) {
        let status = if self.passed { "PASS ✓" } else { "FAIL ✗" };
        println!("\n[{}] {}", status, self.name);
        println!(
            "    Simulation: {:.1}s, {} frames",
            self.duration_s, self.frame_count
        );
        println!("    Assertions:");
        for a in &self.assertions {
            let mark = if a.passed { "✓" } else { "✗" };
            println!(
                "      [{}] {:30} = {:8.4}  (max {:.4})",
                mark, a.metric, a.value, a.max
            );
        }
    }

    pub fn to_csv(&self) -> String {
        let mut out = String::from("time,x,y,z,vx,vy,vz\n");
        for f in &self.frames {
            out.push_str(&format!(
                "{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}\n",
                f.time,
                f.state.position.x,
                f.state.position.y,
                f.state.position.z,
                f.state.velocity.x,
                f.state.velocity.y,
                f.state.velocity.z,
            ));
        }
        out
    }
}
