use std::f32::consts::TAU;

use bevy_math::Vec3;
use geometry::Ray3D;

fn probability_density_function_of_formation(
    value: Vec3,
    template_value: Vec3,
    template_part: Vec3,
    std_dev: f32,
) -> f32 {
    let a = (std_dev * TAU.sqrt()).recip();
    let diff = value.distance(template_part) - value.distance(template_value);
    let b = -diff.powi(2) / (2.0_f32 * std_dev.powi(2));

    a * b.exp()
}

fn combine(values: &[&[Vec3]], coefficients: &[f32]) -> Vec<Vec3> {
    let mut result = Vec::new();

    for i in 0..values[0].len() {
        let mut sum = Vec3::ZERO;

        for j in 0..values.len() {
            sum += values[j][i] * coefficients[j];
        }

        result.push(sum);
    }

    result
}

pub fn expectation_maximization_1d(
    values: &[Vec3],
    formation_templates: &[&[Vec3]],
    max_steps: usize,
) -> Vec<f32> {
    let n_templates = formation_templates.len();
    let n_values = values.len();

    // Initialization step
    let mut coefficients = vec![1.0 / n_templates as f32; n_templates];
    let mut std_deviation = 1.0_f32;

    let mut steps = 0;
    loop {
        // Calculate probabilities of each value belonging to each Gaussian
        let mut probabilities = Vec::new();

        let formation_parts_on_current_coefficients = formation_templates
            .iter()
            .enumerate()
            .map(|(i, formation_template)| combine(&[formation_template], &[coefficients[i]]))
            .collect::<Vec<Vec<Vec3>>>();

        for i in 0..n_values {
            let value = values[i];

            let mut total = 0.0;
            for k in 0..n_templates {
                let formation_parts = &formation_parts_on_current_coefficients[k];

                let a = probability_density_function_of_formation(
                    value,
                    formation_parts[i],
                    formation_templates[k][i],
                    std_deviation,
                ) + 10e-6;

                total += a;

                probabilities.push(a); // Add a small value to avoid division by zero
            }

            if total.abs() > f32::EPSILON {
                for k in 0..n_templates {
                    probabilities[i * n_templates + k] /= total;
                }
            }
        }

        // Update means
        for k in 0..n_templates {
            let mut coefficient = 0.0;
            let mut denominator = 0.0;

            for i in 0..values.len() {
                let ideal_parameter = if formation_templates[k][i].length_squared() < f32::EPSILON {
                    0.0
                } else {
                    let line = Ray3D::new(Vec3::ZERO, formation_templates[k][i]);
                    line.parameter_at_point(values[i])
                };

                coefficient += ideal_parameter * probabilities[i * n_templates + k];
                denominator += probabilities[i * n_templates + k];
            }

            coefficients[k] = coefficient / denominator;
        }

        //clamp coefficients
        coefficients.iter_mut().for_each(|c| *c = c.abs());

        // Normalize coefficients
        let sum: f32 = coefficients.iter().sum();
        coefficients.iter_mut().for_each(|c| *c /= sum);

        // Update standard deviation
        let combined_values = combine(formation_templates, &coefficients);

        let previous_std_deviation = std_deviation;
        std_deviation = (combined_values
            .iter()
            .zip(values.iter())
            .map(|(a, b)| a.distance_squared(*b))
            .sum::<f32>()
            / values.len() as f32)
            .sqrt();

        if std_deviation < f32::EPSILON {
            break;
        }

        if (std_deviation - previous_std_deviation).abs() < 10e-6 {
            break;
        }

        println!(
            "Step: {}, Coefficients: {:?}, Std Deviation: {}",
            steps, coefficients, std_deviation
        );

        steps += 1;

        if steps >= max_steps {
            break;
        }
    }

    coefficients
}

#[cfg(test)]
mod tests {
    use rand::Rng;

    use crate::least_squares::least_squares;

    use super::*;

    fn combine<const T: usize, const P: usize>(
        values: &[[Vec3; P]; T],
        coefficients: &[f32; T],
    ) -> Vec<Vec3> {
        let mut result = Vec::new();

        for i in 0..P {
            let mut sum = Vec3::ZERO;

            for j in 0..T {
                sum += values[j][i] * coefficients[j];
            }

            result.push(sum);
        }

        result
    }

    fn combine_with_randomness<const T: usize, const P: usize>(
        values: &[[Vec3; P]; T],
        coefficients: &[f32; T],
        rng: &mut impl Rng,
        randomness: f32,
    ) -> Vec<Vec3> {
        let mut result = Vec::new();

        for i in 0..P {
            let mut sum = Vec3::ZERO;

            for j in 0..T {
                sum += values[j][i] * coefficients[j]
                    + Vec3::new(
                        rng.gen_range(-randomness..randomness),
                        rng.gen_range(-randomness..randomness),
                        rng.gen_range(-randomness..randomness),
                    );
            }

            result.push(sum);
        }

        result
    }

    #[test]
    fn test_probability_density_function_of_gaussian() {
        let formation_templates = [
            [
                Vec3::new(-100.0, 0.0, 0.0),
                Vec3::new(-80.0, 0.0, 0.0),
                Vec3::new(-60.0, 0.0, 0.0),
                Vec3::new(-40.0, 0.0, 0.0),
                Vec3::new(-20.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(20.0, 0.0, 0.0),
                Vec3::new(40.0, 0.0, 0.0),
                Vec3::new(60.0, 0.0, 0.0),
                Vec3::new(80.0, 0.0, 0.0),
                Vec3::new(100.0, 0.0, 0.0),
            ],
            [
                Vec3::new(0.0, 0.0, -100.0),
                Vec3::new(0.0, 0.0, -80.0),
                Vec3::new(0.0, 0.0, -60.0),
                Vec3::new(0.0, 0.0, -40.0),
                Vec3::new(0.0, 0.0, -20.0),
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 20.0),
                Vec3::new(0.0, 0.0, 40.0),
                Vec3::new(0.0, 0.0, 60.0),
                Vec3::new(0.0, 0.0, 80.0),
                Vec3::new(0.0, 0.0, 100.0),
            ],
            [
                Vec3::new(0.0, -100.0, 0.0),
                Vec3::new(0.0, -80.0, 0.0),
                Vec3::new(0.0, -60.0, 0.0),
                Vec3::new(0.0, -40.0, 0.0),
                Vec3::new(0.0, -20.0, 0.0),
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 20.0, 0.0),
                Vec3::new(0.0, 40.0, 0.0),
                Vec3::new(0.0, 60.0, 0.0),
                Vec3::new(0.0, 80.0, 0.0),
                Vec3::new(0.0, 100.0, 0.0),
            ],
        ];

        let mut rng = rand::thread_rng();
        let values =
            combine_with_randomness(&formation_templates, &[0.4, 0.5, 0.1], &mut rng, 200.0);

        let result = expectation_maximization_1d(
            &values,
            &formation_templates
                .iter()
                .map(|e| e.as_slice())
                .collect::<Vec<&[Vec3]>>(),
            20000,
        );

        let least_squares_result = least_squares(
            &values,
            &formation_templates
                .iter()
                .map(|e| e.as_slice())
                .collect::<Vec<&[Vec3]>>(),
        );

        println!("Least Squares Result: {:?}", least_squares_result);

        assert_eq!(result.len(), 2);
        panic!("{:?}", result);
    }
}
