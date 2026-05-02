/// RoveControl protobuf messages — manually derived with prost to avoid
/// needing `protoc` at build time.  Mirrors `proto/RoveControl.proto` and
/// `proto/core/JointState.proto`.

#[derive(Clone, PartialEq, prost::Message)]
pub struct RoveControl {
    #[prost(message, optional, tag = "1")]
    pub tracks: Option<Tracks>,
    #[prost(message, optional, tag = "2")]
    pub flippers: Option<Flippers>,
    #[prost(message, optional, tag = "3")]
    pub ovis: Option<Ovis>,
    #[prost(uint64, tag = "4")]
    pub timestamp_us: u64,
}

/// Normalized track velocities (-1.0 .. 1.0).
#[derive(Clone, PartialEq, prost::Message)]
pub struct Tracks {
    #[prost(float, tag = "1")]
    pub left_vel: f32,
    #[prost(float, tag = "2")]
    pub right_vel: f32,
}

#[derive(Clone, PartialEq, prost::Message)]
pub struct JointState {
    #[prost(float, tag = "1")]
    pub vel: f32,
    #[prost(float, tag = "2")]
    pub pos_deg: f32,
    #[prost(float, tag = "3")]
    pub torque: f32,
}

#[derive(Clone, PartialEq, prost::Message)]
pub struct Flippers {
    #[prost(message, optional, tag = "1")]
    pub fl: Option<JointState>,
    #[prost(message, optional, tag = "2")]
    pub fr: Option<JointState>,
    #[prost(message, optional, tag = "3")]
    pub rl: Option<JointState>,
    #[prost(message, optional, tag = "4")]
    pub rr: Option<JointState>,
}

#[derive(Clone, PartialEq, prost::Message)]
pub struct Ovis {
    #[prost(message, optional, tag = "1")]
    pub act_1: Option<JointState>,
    #[prost(message, optional, tag = "2")]
    pub act_2: Option<JointState>,
    #[prost(message, optional, tag = "3")]
    pub act_3: Option<JointState>,
    #[prost(message, optional, tag = "4")]
    pub act_4: Option<JointState>,
    #[prost(message, optional, tag = "5")]
    pub act_5: Option<JointState>,
    #[prost(message, optional, tag = "6")]
    pub act_6: Option<JointState>,
}
