use std::{fmt::Debug, ops::Mul};

use crate::EPSILON;

#[derive(Clone)]
struct MatrixData {
    data: Vec<f32>,
    rows: usize,
    cols: usize,
    transposed: bool,
}

impl MatrixData {
    fn new(data: Vec<f32>, cols: usize, rows: usize) -> Self {
        MatrixData {
            data,
            cols,
            rows,
            transposed: false,
        }
    }

    fn get_unchecked(&self, row: usize, col: usize) -> f32 {
        if self.transposed {
            self.data[(col * self.cols + row) % self.data.len()]
        } else {
            self.data[(row * self.cols + col) % self.data.len()]
        }
    }

    fn set_unchecked(&mut self, row: usize, col: usize, value: f32) {
        let len = self.data.len();
        if self.transposed {
            self.data[(col * self.cols + row) % len] = value;
        } else {
            self.data[(row * self.cols + col) % len] = value;
        }
    }

    fn get(&self, row: usize, col: usize) -> Option<f32> {
        if row >= self.rows() || col >= self.cols() {
            return None;
        }
        Some(self.get_unchecked(row, col))
    }

    fn set(&mut self, row: usize, col: usize, value: f32) -> Option<()> {
        if row >= self.rows() || col >= self.cols() {
            return None;
        }
        self.set_unchecked(row, col, value);
        Some(())
    }

    fn transpose(&mut self) {
        self.transposed = !self.transposed;
    }

    fn rows(&self) -> usize {
        if self.transposed {
            self.cols
        } else {
            self.rows
        }
    }

    fn cols(&self) -> usize {
        if self.transposed {
            self.rows
        } else {
            self.cols
        }
    }

    fn swap(&mut self, (row1, col1): (usize, usize), (row2, col2): (usize, usize)) {
        let temp = self.get_unchecked(row1, col1);
        self.set_unchecked(row1, col1, self.get_unchecked(row2, col2));
        self.set_unchecked(row2, col2, temp);
    }
}

#[derive(Clone)]
pub struct Matrix {
    data: MatrixData,
}

impl Matrix {
    pub fn rows(&self) -> usize {
        self.data.rows()
    }

    pub fn cols(&self) -> usize {
        self.data.cols()
    }

    // Example method to create a new matrix
    pub fn from_slice<const COLS: usize, const ROWS: usize>(rows: [[f32; COLS]; ROWS]) -> Self {
        let mut data_vec = Vec::new();
        for row in rows.iter().take(ROWS) {
            data_vec.append(row.to_vec().as_mut());
        }
        Matrix {
            data: MatrixData::new(data_vec, COLS, ROWS),
        }
    }

    pub fn new(data: Vec<f32>, cols: usize, rows: usize) -> Self {
        Matrix {
            data: MatrixData::new(data, cols, rows),
        }
    }

    pub fn empty(cols: usize, rows: usize) -> Self {
        Matrix {
            data: MatrixData::new(vec![0.0; cols * rows], cols, rows),
        }
    }

    pub fn identity(size: usize) -> Self {
        let mut data = vec![0.0; size * size];
        for i in 0..size {
            data[i * size + i] = 1.0;
        }
        Matrix {
            data: MatrixData::new(data, size, size),
        }
    }

    pub fn get(&self, row: usize, col: usize) -> Option<f32> {
        self.data.get(row, col)
    }

    pub fn mul_left(&self, rhs: &Matrix) -> Option<Matrix> {
        if self.data.cols() != rhs.data.rows() {
            return None;
        }

        let mut data = vec![0.0; self.data.rows() * rhs.data.cols()];

        for i in 0..self.data.rows() {
            for j in 0..rhs.data.cols() {
                for k in 0..self.data.cols() {
                    let a = self.data.get_unchecked(i, k);
                    let b = rhs.data.get_unchecked(k, j);
                    data[i * rhs.data.cols() + j] += a * b;
                }
            }
        }

        Some(Matrix::new(data, rhs.data.cols(), self.data.rows()))
    }

    pub fn inverse(&self) -> Option<Matrix> {
        // Check if matrix is square
        if self.data.cols() != self.data.rows() {
            return None;
        }

        let det = self.determinant()?;

        if det.abs() < EPSILON {
            return None;
        }

        let mut inverse = vec![0.0; self.data.rows() * self.data.cols()];

        if self.data.cols() == 2 {
            let inv_det = 1.0 / det;

            inverse[0] = self.data.get_unchecked(1, 1) * inv_det;
            inverse[1] = -self.data.get_unchecked(0, 1) * inv_det;
            inverse[2] = -self.data.get_unchecked(1, 0) * inv_det;
            inverse[3] = self.data.get_unchecked(0, 0) * inv_det;

            return Some(Matrix::new(inverse, self.data.cols(), self.data.rows()));
        }

        let mut adjugate = vec![0.0; self.data.rows() * self.data.cols()];
        let mut minor_matrix = Matrix::empty(self.data.cols() - 1, self.data.rows() - 1);

        for i in 0..self.data.rows() {
            for j in 0..self.data.cols() {
                let mut minor_row = 0;
                for k in 0..self.data.rows() {
                    if k == i {
                        continue;
                    }
                    let mut minor_col = 0;
                    for l in 0..self.data.cols() {
                        if l == j {
                            continue;
                        }
                        minor_matrix.data.set_unchecked(
                            minor_row,
                            minor_col,
                            self.data.get_unchecked(k, l),
                        );
                        minor_col += 1;
                    }
                    minor_row += 1;
                }

                let minor_det = minor_matrix.determinant()?;
                adjugate[i * self.data.cols() + j] = if (i + j) % 2 == 0 {
                    minor_det
                } else {
                    -minor_det
                };
            }
        }

        for i in 0..self.data.rows() {
            for j in 0..self.data.cols() {
                inverse[j * self.data.cols() + i] = adjugate[i * self.data.cols() + j] / det;
            }
        }

        Some(Matrix::new(inverse, self.data.cols(), self.data.rows()))
    }

    pub fn transpose(&mut self) {
        self.data.transpose();
    }

    pub fn transposed(&self) -> Matrix {
        let mut clone = self.clone();

        clone.transpose();

        clone
    }

    pub fn determinant(&self) -> Option<f32> {
        // Check if matrix is square
        if self.data.cols() != self.data.rows() {
            return None;
        }

        if self.data.cols() == 2 {
            return Some(
                self.data.get_unchecked(0, 0) * self.data.get_unchecked(1, 1)
                    - self.data.get_unchecked(0, 1) * self.data.get_unchecked(1, 0),
            );
        }

        let (lower, upper, permutation_sign) = match self.lu_decomposition() {
            Some((lower, upper, permuation_sign)) => (lower, upper, permuation_sign),
            None => return None,
        };

        let mut det = permutation_sign;
        for i in 0..self.data.cols() {
            det *= lower.data.get_unchecked(i, i) * upper.data.get_unchecked(i, i);
        }

        Some(det)
    }

    pub fn lu_decomposition(&self) -> Option<(Matrix, Matrix, f32)> {
        // Check if matrix is square
        if self.data.cols() != self.data.rows() {
            return None;
        }

        let mut lower = Matrix::empty(self.data.cols(), self.data.rows());
        let mut upper = Matrix::empty(self.data.cols(), self.data.rows());
        let mut permutation = vec![0; self.data.cols()];
        let mut permutation_sign = 1;

        let mut data = self.data.clone();

        #[allow(clippy::needless_range_loop)]
        for i in 0..self.data.cols() {
            permutation[i] = i;
        }

        // Gaussian elimination with partial pivoting (LU decomposition)
        for i in 0..self.data.cols() {
            let mut max = 0.0;
            let mut max_row = i;
            for k in i..self.data.cols() {
                let abs = data.get_unchecked(k, i).abs();
                if abs > max {
                    max = abs;
                    max_row = k;
                }
            }

            if max <= EPSILON {
                return None;
            }

            if max_row != i {
                for j in 0..self.data.cols() {
                    data.swap((i, j), (max_row, j));
                }
                permutation.swap(i, max_row);
                permutation_sign = -permutation_sign;
            }

            for j in i..self.data.cols() {
                upper.data.set_unchecked(i, j, data.get_unchecked(i, j));
            }

            for j in (i + 1)..self.data.cols() {
                lower.data.set_unchecked(
                    j,
                    i,
                    data.get_unchecked(j, i) / upper.data.get_unchecked(i, i),
                );
                for k in i..self.data.cols() {
                    data.set_unchecked(
                        j,
                        k,
                        data.get_unchecked(j, k)
                            - lower.data.get_unchecked(j, i) * upper.data.get_unchecked(i, k),
                    );
                }
            }
        }

        // Ensure the diagonal of L is set to 1
        for i in 0..self.data.cols() {
            lower.data.set_unchecked(i, i, 1.0);
        }

        Some((lower, upper, permutation_sign as f32))
    }
    pub fn qr_decompose(&self) -> (Matrix, Matrix) {
        let m = self.data.rows();
        let n = self.data.cols();
        let mut q = vec![0.0; m * n];
        let mut r = vec![0.0; n * n];

        let mut a_columns: Vec<Vec<f32>> = (0..n)
            .map(|j| (0..m).filter_map(|i| self.get(i, j)).collect())
            .collect();

        for k in 0..n {
            let mut norm = 0.0;
            for i in 0..m {
                norm += a_columns[k][i] * a_columns[k][i];
            }
            norm = norm.sqrt();
            r[k * n + k] = norm;

            for i in 0..m {
                q[i * n + k] = a_columns[k][i] / norm;
            }

            for j in (k + 1)..n {
                let mut dot = 0.0;
                for i in 0..m {
                    dot += q[i * n + k] * a_columns[j][i];
                }
                r[k * n + j] = dot;

                for i in 0..m {
                    a_columns[j][i] -= dot * q[i * n + k];
                }
            }
        }

        let q_matrix = Matrix::new(q, m, n);
        let r_matrix = Matrix::new(r, n, n);
        (q_matrix, r_matrix)
    }

    pub fn invert_upper_triangular(&self) -> Option<Matrix> {
        if self.data.cols() != self.data.rows() {
            return None;
        }

        let n = self.data.rows();
        let mut inv = vec![0.0; n * n];

        for i in (0..n).rev() {
            inv[i * n + i] = 1.0 / self.data.get_unchecked(i, i);
            for j in (0..i).rev() {
                let mut sum = 0.0;
                for k in (j + 1)..=i {
                    sum += self.data.get_unchecked(j, k) * inv[k * n + i];
                }
                inv[j * n + i] = -sum / self.data.get_unchecked(j, j);
            }
        }

        Some(Matrix::new(inv, n, n))
    }

    pub fn pseudoinverse(&self) -> Matrix {
        let (mut q, r) = self.qr_decompose();

        // Invert R
        let r_inv = r
            .invert_upper_triangular()
            .expect("R matrix from QR decomposition isn't a square matrix");

        // Compute Q^T
        q.transpose();

        // Multiply R^{-1} * Q^T
        r_inv.mul_left(&q).expect("Matrix cannot be multiplied")
    }
}

impl Mul<Matrix> for Matrix {
    type Output = Option<Matrix>;

    fn mul(self, rhs: Matrix) -> Self::Output {
        self.mul_left(&rhs)
    }
}

impl Debug for Matrix {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        for i in 0..self.data.rows() {
            write!(f, "[")?;
            for j in 0..self.data.cols() {
                write!(f, "{:.2}, ", self.data.get(i, j).unwrap())?;
            }
            write!(f, "]")?;
            writeln!(f)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use approx::relative_eq;

    use crate::matrix::Matrix;

    #[test]
    fn test_matrix_multiply() {
        let m1 = Matrix::from_slice([[1.0, 2.0], [3.0, 4.0]]);
        let m2 = Matrix::from_slice([[1.0, 2.0], [3.0, 4.0]]);
        let m3 = (m1 * m2).expect("Matrix cannot be multiplied");
        relative_eq!(m3.get(0, 0).unwrap(), 7.0);
        relative_eq!(m3.get(0, 1).unwrap(), 10.0);
        relative_eq!(m3.get(1, 0).unwrap(), 15.0);
        relative_eq!(m3.get(1, 1).unwrap(), 22.0);
    }

    #[test]
    fn test_matrix_inverse2x2() {
        let m1 = Matrix::from_slice([[1.0, 2.0], [3.0, 4.0]]);
        let m2 = m1.inverse().expect("Matrix is not invertible");
        relative_eq!(m2.get(0, 0).unwrap(), -2.0);
        relative_eq!(m2.get(0, 1).unwrap(), 1.0);
        relative_eq!(m2.get(1, 0).unwrap(), 1.5);
        relative_eq!(m2.get(1, 1).unwrap(), -0.5);
    }

    #[test]
    fn test_matrix_inverse3x3() {
        let m1 = Matrix::from_slice([[1.0, 0.0, 5.0], [2.0, 1.0, 6.0], [3.0, 4.0, 0.0]]);
        let m2 = m1.inverse().expect("Matrix is not invertible");
        relative_eq!(m2.get(0, 0).unwrap(), -24.0);
        relative_eq!(m2.get(0, 1).unwrap(), 20.0);
        relative_eq!(m2.get(0, 2).unwrap(), -5.0);
        relative_eq!(m2.get(1, 0).unwrap(), 18.0);
        relative_eq!(m2.get(1, 1).unwrap(), -15.0);
        relative_eq!(m2.get(1, 2).unwrap(), 4.0);
        relative_eq!(m2.get(2, 0).unwrap(), 5.0);
        relative_eq!(m2.get(2, 1).unwrap(), -4.0);
        relative_eq!(m2.get(2, 2).unwrap(), 1.0);
    }

    #[test]
    fn test_matrix_inverse4x4() {
        let m1 = Matrix::from_slice([
            [1.0, 2.0, 3.0, 3.0],
            [3.0, 2.0, 1.0, 1.0],
            [2.0, 1.0, 3.0, 5.0],
            [4.0, 2.0, 4.0, 5.0],
        ]);

        let inverse = m1.inverse().expect("Matrix is not invertible");

        let mul = (inverse * m1).expect("Matrix cannot be multiplied");

        for i in 0..4 {
            for j in 0..4 {
                if i == j {
                    relative_eq!(mul.get(i, j).unwrap(), 1.0);
                } else {
                    relative_eq!(mul.get(i, j).unwrap(), 0.0);
                }
            }
        }
    }

    #[test]
    fn test_matrix_inverse10x10() {
        let m1 = Matrix::from_slice([
            [-1.0, -6.0, 8.0, 3.0, 8.0, -9.0, -9.0, 6.0, 4.0, -5.0],
            [3.0, -2.0, -7.0, 4.0, -3.0, -2.0, -10.0, 9.0, 7.0, -2.0],
            [6.0, -3.0, -10.0, -6.0, 2.0, -2.0, 3.0, -8.0, 3.0, 9.0],
            [5.0, 1.0, -9.0, -5.0, 3.0, -7.0, -5.0, -5.0, 4.0, 6.0],
            [-5.0, -3.0, 2.0, -5.0, -9.0, 7.0, 2.0, -2.0, 3.0, 5.0],
            [1.0, -4.0, -2.0, -3.0, -9.0, -8.0, 1.0, -4.0, 9.0, 5.0],
            [1.0, 7.0, 0.0, -2.0, -6.0, 4.0, -7.0, -8.0, 7.0, -4.0],
            [5.0, 2.0, 8.0, 1.0, -6.0, -4.0, 9.0, 6.0, 2.0, -3.0],
            [7.0, -4.0, -3.0, -1.0, -1.0, -3.0, -7.0, -7.0, -2.0, 8.0],
            [1.0, -5.0, -10.0, -8.0, 5.0, 1.0, 1.0, 3.0, 6.0, -7.0],
        ]);

        let inverse = m1.inverse().expect("Matrix is not invertible");

        let mul = (inverse * m1).expect("Matrix cannot be multiplied");

        for i in 0..4 {
            for j in 0..4 {
                if i == j {
                    relative_eq!(mul.get(i, j).unwrap(), 1.0);
                } else {
                    relative_eq!(mul.get(i, j).unwrap(), 0.0);
                }
            }
        }
    }

    #[test]
    fn test_matrix_transpose() {
        let m1 = Matrix::from_slice([[1.0, 2.0], [3.0, 4.0]]);
        let m2 = m1.transposed();
        relative_eq!(m2.get(0, 0).unwrap(), 1.0);
        relative_eq!(m2.get(0, 1).unwrap(), 3.0);
        relative_eq!(m2.get(1, 0).unwrap(), 2.0);
        relative_eq!(m2.get(1, 1).unwrap(), 4.0);
    }

    #[test]
    fn test_matrix_transpose_non_square() {
        let m1 = Matrix::from_slice([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);

        let m2 = m1.transposed();

        relative_eq!(m2.get(0, 0).unwrap(), 1.0);
        relative_eq!(m2.get(0, 1).unwrap(), 4.0);
        relative_eq!(m2.get(1, 0).unwrap(), 2.0);
        relative_eq!(m2.get(1, 1).unwrap(), 5.0);
        relative_eq!(m2.get(2, 0).unwrap(), 3.0);
        relative_eq!(m2.get(2, 1).unwrap(), 6.0);
    }

    #[test]
    fn test_matrix_determinant2x2() {
        let m1 = Matrix::from_slice([[1.0, 2.0], [3.0, 4.0]]);
        relative_eq!(m1.determinant().unwrap(), -2.0);
    }

    #[test]
    fn test_matrix_determinant3x3() {
        let m1 = Matrix::from_slice([[6.0, 4.0, 2.0], [1.0, -2.0, 8.0], [1.0, 5.0, 7.0]]);
        relative_eq!(m1.determinant().unwrap(), -306.0);
    }

    #[test]
    fn test_qr_decompose3x3() {
        let m1 = Matrix::from_slice([[12.0, -51.0, 4.0], [6.0, 167.0, -68.0], [-4.0, 24.0, -41.0]]);

        let (q, r) = m1.qr_decompose();

        relative_eq!(q.get(0, 0).unwrap(), 6.0 / 7.0);
        relative_eq!(q.get(0, 1).unwrap(), -69.0 / 175.0);
        relative_eq!(q.get(0, 2).unwrap(), -58.0 / 175.0);
        relative_eq!(q.get(1, 0).unwrap(), 3.0 / 7.0);
        relative_eq!(q.get(1, 1).unwrap(), 158.0 / 175.0);
        relative_eq!(q.get(1, 2).unwrap(), 6.0 / 175.0);
        relative_eq!(q.get(2, 0).unwrap(), -2.0 / 7.0);
        relative_eq!(q.get(2, 1).unwrap(), 6.0 / 35.0);
        relative_eq!(q.get(2, 2).unwrap(), -33.0 / 35.0);

        relative_eq!(r.get(0, 0).unwrap(), 14.0);
        relative_eq!(r.get(0, 1).unwrap(), 21.0);
        relative_eq!(r.get(0, 2).unwrap(), 14.0);
        relative_eq!(r.get(1, 0).unwrap(), 0.0);
        relative_eq!(r.get(1, 1).unwrap(), 175.0);
        relative_eq!(r.get(1, 2).unwrap(), -70.0);
        relative_eq!(r.get(2, 0).unwrap(), 0.0);
        relative_eq!(r.get(2, 1).unwrap(), 0.0);
        relative_eq!(r.get(2, 2).unwrap(), 35.0);
    }

    #[test]
    fn test_pseudoinverse2x2() {
        let m1 = Matrix::from_slice([[1.0, 2.0], [3.0, 4.0]]);
        let m2 = m1.pseudoinverse();

        let m3 = (m1.clone() * m2).expect("Matrix cannot be multiplied");

        for i in 0..2 {
            for j in 0..2 {
                if i == j {
                    relative_eq!(m3.get(i, j).unwrap(), 1.0);
                } else {
                    relative_eq!(m3.get(i, j).unwrap(), 0.0);
                }
            }
        }
    }

    #[test]
    fn test_pseudoinverse3x3() {
        let m1 = Matrix::from_slice([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
        let m2 = m1.pseudoinverse();

        let m3 = (m1.clone() * m2).expect("Matrix cannot be multiplied");

        for i in 0..3 {
            for j in 0..3 {
                if i == j {
                    relative_eq!(m3.get(i, j).unwrap(), 1.0);
                } else {
                    relative_eq!(m3.get(i, j).unwrap(), 0.0);
                }
            }
        }
    }

    #[test]
    fn test_pseudoinverse4x4() {
        let m1 = Matrix::from_slice([
            [-1.0, -6.0, 8.0, 3.0, 8.0, -9.0, -9.0, 6.0, 4.0, -5.0],
            [3.0, -2.0, -7.0, 4.0, -3.0, -2.0, -10.0, 9.0, 7.0, -2.0],
            [6.0, -3.0, -10.0, -6.0, 2.0, -2.0, 3.0, -8.0, 3.0, 9.0],
            [5.0, 1.0, -9.0, -5.0, 3.0, -7.0, -5.0, -5.0, 4.0, 6.0],
            [-5.0, -3.0, 2.0, -5.0, -9.0, 7.0, 2.0, -2.0, 3.0, 5.0],
            [1.0, -4.0, -2.0, -3.0, -9.0, -8.0, 1.0, -4.0, 9.0, 5.0],
            [1.0, 7.0, 0.0, -2.0, -6.0, 4.0, -7.0, -8.0, 7.0, -4.0],
            [5.0, 2.0, 8.0, 1.0, -6.0, -4.0, 9.0, 6.0, 2.0, -3.0],
            [7.0, -4.0, -3.0, -1.0, -1.0, -3.0, -7.0, -7.0, -2.0, 8.0],
            [1.0, -5.0, -10.0, -8.0, 5.0, 1.0, 1.0, 3.0, 6.0, -7.0],
        ]);

        let inverse = m1.pseudoinverse();

        let mul = (inverse * m1).expect("Matrix cannot be multiplied");

        for i in 0..4 {
            for j in 0..4 {
                if i == j {
                    relative_eq!(mul.get(i, j).unwrap(), 1.0);
                } else {
                    relative_eq!(mul.get(i, j).unwrap(), 0.0);
                }
            }
        }
    }
}
