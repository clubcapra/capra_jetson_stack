//! Shared state shapes that flow across the IK / kinova / teleop tasks.

use serde_json::Value;

#[derive(Debug, Clone, Default)]
pub struct ArmSnapshot {
    pub joint_pos_deg: [f32; 6],
    pub joint_vel_deg_s: [f32; 6],
    pub joint_current_a: [f32; 6],
    pub joint_temp_c: [f32; 6],
    pub bus_voltage: f32,
    pub bus_current: f32,
    pub timestamp_ns: i64,
}

impl ArmSnapshot {
    /// Pull joint fields out of a kinova_arm /data JSON. Returns None if
    /// none of the joint_*_pos fields are present (treat as a non-data
    /// frame).
    pub fn from_kinova_json(v: &Value) -> Option<Self> {
        let mut s = Self::default();
        let mut got_any = false;
        for i in 0..6 {
            let p = v.get(format!("joint_{}_pos", i + 1)).and_then(Value::as_f64);
            let q = v.get(format!("joint_{}_vel", i + 1)).and_then(Value::as_f64);
            let c = v
                .get(format!("joint_{}_current", i + 1))
                .and_then(Value::as_f64);
            let t = v.get(format!("joint_{}_temp", i + 1)).and_then(Value::as_f64);
            if let Some(p) = p {
                s.joint_pos_deg[i] = p as f32;
                got_any = true;
            }
            if let Some(q) = q {
                s.joint_vel_deg_s[i] = q as f32;
            }
            if let Some(c) = c {
                s.joint_current_a[i] = c as f32;
            }
            if let Some(t) = t {
                s.joint_temp_c[i] = t as f32;
            }
        }
        if !got_any {
            return None;
        }
        s.bus_voltage = v.get("bus_voltage").and_then(Value::as_f64).unwrap_or(0.0) as f32;
        s.bus_current = v.get("bus_current").and_then(Value::as_f64).unwrap_or(0.0) as f32;
        s.timestamp_ns = v.get("timestamp_ns").and_then(Value::as_i64).unwrap_or(0);
        Some(s)
    }
}
