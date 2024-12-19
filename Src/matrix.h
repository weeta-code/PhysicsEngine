#ifndef CUSTOM_PHYSICS_ENGINE_MATRIX_H
#define CUSTOM_PHYSICS_ENGINE_MATRIX_H

#include "utilities.h"
#include <assert.h>

namespace vic_scs {
    class Matrix {
        public: 
        Matrix();
        Matrix(int width, int height, double value = 0.0);
        ~Matrix();

        void initialize(int width, int height, double value);
        void initialize(int width, int height);
        void resize(int width, int height);
        void destroy();

        void set(const double *data);

        scs_force_inline void set(int col, int row, double value) {
            assert(col >= 0 && col < m_width);
            assert(row >= 0 && row < m_height);

            m_matrix[row][col] = value;
        }

        scs_force_inline void add(int col, int row, double value) {
            assert(col >= 0 && col < m_width);
            assert(row >= 0 && row < m_height);

            m_matrix[row][col] += value;
        }

        scs_force_inline double get(int col, int row) {
            assert(col >= 0 && col < m_width);
            assert(row >= 0 && row < m_height);

            return m_matrix[row][col];
        }

        set

        protected:
            double **m_matrix;
            double *m_data;
            int m_width;
            int m_height;
            int m_capacityWidth;
            int m_capacityHeight;
    };
}

#endif