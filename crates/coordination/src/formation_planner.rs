use bevy_math::Vec3;

use crate::{least_squares::least_squares, Formation, FormationTemplate};

pub struct FormationTemplateSet(Vec<&'static dyn FormationTemplate>);

struct FormationSet(Vec<Formation>);

impl FormationTemplateSet {}

impl FormationSet {
    /// Combine the formations in the set into a single formation given a slice of coefficients
    fn combine_into_formation(&self, coefficients: &[f32]) -> Formation {
        assert_eq!(self.0.len(), coefficients.len());

        let formation_size = self.0[0].get_positions().len();

        let mut positions = Vec::with_capacity(formation_size);
        for i in 0..formation_size {
            let mut position = Vec3::ZERO;
            for (j, formation) in self.0.iter().enumerate() {
                position += formation.get_positions()[i] * coefficients[j];
            }
            positions.push(position);
        }

        Formation::new(positions)
    }
}
