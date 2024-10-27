mod circle_formation;
mod expectation_maximization;
mod formation;
mod formation_template;
mod hungarian;
mod least_squares;
mod line_formation;
mod queue_formation;
mod v_formation;

pub use expectation_maximization::best_matching_indexes;
pub use formation::*;
pub use formation_template::*;

pub mod formations {
    pub use crate::circle_formation::CircleFormation;
    pub use crate::line_formation::LineFormation;
    pub use crate::queue_formation::QueueFormation;
    pub use crate::v_formation::VFormation;
}
