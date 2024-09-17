use bevy_math::Vec3;
use geometry::Matrix;

pub fn least_squares(measured_values: &[Vec3], independent_variables: &[&[Vec3]]) -> Matrix {
    let f_matrix = Matrix::new(
        measured_values
            .iter()
            .flat_map(|v| [v.x, v.y, v.z])
            .collect::<Vec<_>>(),
        1,
        measured_values.len() * 3,
    );

    let t_matrix = {
        let num_rows = 3 * measured_values.len();
        let num_cols = independent_variables.len();
        let mut data = Vec::with_capacity(num_rows * num_cols);

        for i in 0..measured_values.len() {
            // Collect x_column elements and append to data
            for &v in independent_variables.iter() {
                data.push(v[i].x);
            }
            // Collect y_column elements and append to data
            for &v in independent_variables.iter() {
                data.push(v[i].y);
            }

            // Collect z_column elements and append to data
            for &v in independent_variables.iter() {
                data.push(v[i].z);
            }
        }

        Matrix::new(data, num_cols, num_rows)
    };

    let t_transposed = t_matrix.transposed();

    let a = t_transposed
        .mul_left(&t_matrix)
        .expect("Unable to multiple transposed matrix with original matrix");

    let a_inv = a.pseudoinverse();

    let b = a_inv
        .mul_left(&t_transposed)
        .expect("Unable to multiple inverse matrix with transposed matrix");

    b.mul_left(&f_matrix)
        .expect("Unable to multiple b matrix with f matrix")
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::relative_eq;

    fn combine<const Vals: usize, const Templates: usize>(
        weights: [f32; Templates],
        values: [[Vec3; Vals]; Templates],
    ) -> [Vec3; Vals] {
        let mut result = [Vec3::ZERO; Vals];
        for i in 0..Vals {
            for j in 0..Templates {
                result[i] += values[j][i] * weights[j];
            }
        }
        result
    }

    #[test]
    fn test_least_squares() {
        let independent_variables = [
            [
                Vec3::new(0.0, -30.0, 0.0),
                Vec3::new(0.0, -15.0, 0.0),
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 15.0, 0.0),
                Vec3::new(0.0, 30.0, 0.0),
            ],
            [
                Vec3::new(-20.0, -20.0, 0.0),
                Vec3::new(-10.0, -10.0, 0.0),
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(-10.0, 10.0, 0.0),
                Vec3::new(-20.0, 20.0, 0.0),
            ],
            [
                Vec3::new(-30.0, 0.0, 0.0),
                Vec3::new(-15.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(-45.0, 0.0, 0.0),
                Vec3::new(-60.0, 0.0, 0.0),
            ],
        ];

        let measured_values = combine([1.7, 4.1, 1.3], independent_variables);
        let result = least_squares(
            &measured_values,
            independent_variables
                .iter()
                .map(|v| v.as_slice())
                .collect::<Vec<_>>()
                .as_slice(),
        );

        relative_eq!(result.get(0, 0).unwrap(), 1.7);
        relative_eq!(result.get(1, 0).unwrap(), 4.1);
        relative_eq!(result.get(2, 0).unwrap(), 1.3);
    }
}
