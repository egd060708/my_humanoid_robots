/*! @file	myMatrices.h
 *  @brief	基于一维向量的矩阵运算类型
 *	@author	zzr
 *  @date	2023.9.10
 *
 *
 */
#ifndef _MYMATRICES_H_
#define _MYMATRICES_H_

#include <iostream>
#include <vector>
#include <stdexcept>

#define USING_NAMESPACE_MM using namespace mM;
#define MATRIX myMatrices
#define MATRIX_PTR myMatrices *

namespace mM
{

    class myMatrices
    {
        /* 友元函数 */
        friend myMatrices rowCombine(myMatrices &A, myMatrices &B);
        template <class... Args>
        friend myMatrices rowCombine(myMatrices &A, myMatrices &B, Args... rest);
        friend myMatrices colCombine(myMatrices &A, myMatrices &B);
        template <class... Args>
        friend myMatrices colCombine(myMatrices &A, myMatrices &B, Args... rest);

    private:
        uint16_t rows;
        uint16_t cols;
        // float data[16];
        float *data;

    public:
        /*析构函数*/
        ~myMatrices()
        {
            delete[] data;
        }
        /* 行列构造 */
        myMatrices(uint16_t rows, uint16_t cols) : rows(rows), cols(cols)
        {
            data = new float[rows * cols];
            // clear(0);
        }
        /* 方阵构造 */
        myMatrices(uint16_t n) : rows(n), cols(n)
        {
            data = new float[rows * cols];
            // clear(0);
        }

        /* 矩阵清理（赋值统一） */
        void clear(float num)
        {
            if (rows > 0 && cols > 0)
            {
                for (uint16_t i = 0; i < rows; i++)
                {
                    for (uint16_t j = 0; j < cols; j++)
                    {
                        data[i * cols + j] = num;
                    }
                }
            }
        }

        /* 单位矩阵 */
        void eye()
        {
            if ((rows > 0) && (cols > 0) && (rows == cols))
            {
                for (uint16_t i = 0; i < rows; i++)
                {
                    data[i * cols + i] = 1;
                }
            }
        }

        /* 获取矩阵的行数 */
        uint16_t getRows() const
        {
            return rows;
        }

        /* 获取矩阵的列数 */
        uint16_t getCols() const
        {
            return cols;
        }

        /* 获取矩阵中特定位置的元素 */
        float getElement(uint16_t row, uint16_t col) const
        {
            return data[row * cols + col];
        }

        /* 设置矩阵中特定位置的元素 */
        void setElement(uint16_t row, uint16_t col, float value)
        {
            data[row * cols + col] = value;
        }

        /* 设置整个矩阵向量 */
        void setArray(const float *array, uint16_t length)
        {
            if (rows == length / cols)
            {
                memcpy(data, array, rows * cols * sizeof(float));
            }
            else
            {
            }
        }

        /* 设置行向量 */
        void setRowArray(const float *_array, uint16_t _row)
        {
            if (_row < rows)
            {
                for (int i = 0; i < cols; i++)
                {
                    data[cols * _row + i] = _array[i]; //取出对应行数上的每一个元素
                }
            }
        }

        /* 设置列向量 */
        void setColArray(const float *_array, uint16_t _col)
        {
            if (_col < cols)
            {
                for (int i = 0; i < rows; i++)
                {
                    data[i * cols + _col] = _array[i]; //取出对应列数上的每一个元素
                }
            }
        }

        /* 返回默认数据类型的数据向量 */
        const float *getArray()
        {
            return data;
        }

        /* 返回行向量 */
        void getRowArray(float *_dst, uint16_t _row)
        {
            if (_row < rows)
            {
                for (int i = 0; i < cols; i++)
                {
                    _dst[i] = data[cols * _row + i]; //取出对应行数上的每一个元素
                }
            }
        }

        /* 返回列向量 */
        void getColArray(float *_dst, uint16_t _col)
        {
            if (_col < cols)
            {
                for (int i = 0; i < rows; i++)
                {
                    _dst[i] = data[i * cols + _col]; //取出对应列数上的每一个元素
                }
            }
        }

        /* 打印矩阵 */
        void print() const
        {
            for (uint16_t i = 0; i < rows; i++)
            {
                for (uint16_t j = 0; j < cols; j++)
                {
                    std::cout << getElement(i, j) << " ";
                }
                std::cout << std::endl;
            }
            std::cout << std::endl;
        }

        //矩阵加法
        myMatrices operator+(const myMatrices &other) const;
        // 矩阵减法
        myMatrices operator-(const myMatrices &other) const;
        // 矩阵乘法
        myMatrices operator*(const myMatrices &other) const;
        myMatrices operator*(const float other) const;
        // 矩阵求行列式
        float determinant() const;
        // 矩阵求代数余子式
        myMatrices getCofactorMatrix(uint16_t rowToRemove, uint16_t colToRemove) const;
        // 矩阵转置
        myMatrices transpose() const;
        // 矩阵求逆
        myMatrices inverse() const;
        // 矩阵行增广
        void rowExpansion(myMatrices other);
        // 矩阵列增广
        void colExpansion(myMatrices other);
        // 旋转齐次变换矩阵
        void rotateX_T(float theta);
        void rotateY_T(float theta);
        void rotateZ_T(float theta);
        // 平移齐次变换矩阵
        void transform_T(float x,float y,float z);
    };

    /* 旋转齐次变换矩阵 */
    inline void myMatrices::rotateX_T(float theta)
    {
        rows = 4;
        cols = 4;
        setElement(0,0,1);
        setElement(0,1,0);
        setElement(0,2,0);
        setElement(0,3,0);
        setElement(1,0,0);
        setElement(1,1,cosf(theta));
        setElement(1,2,-sinf(theta));
        setElement(1,3,0);
        setElement(2,0,0);
        setElement(2,1,sinf(theta));
        setElement(2,2,cosf(theta));
        setElement(2,3,0);
        setElement(3,0,0);
        setElement(3,1,0);
        setElement(3,2,0);
        setElement(3,3,1);
    }

    inline void myMatrices::rotateY_T(float theta)
    {
        rows = 4;
        cols = 4;
        setElement(0,0,cosf(theta));
        setElement(0,1,0);
        setElement(0,2,sinf(theta));
        setElement(0,3,0);
        setElement(1,0,0);
        setElement(1,1,1);
        setElement(1,2,0);
        setElement(1,3,0);
        setElement(2,0,-sinf(theta));
        setElement(2,1,0);
        setElement(2,2,cosf(theta));
        setElement(2,3,0);
        setElement(3,0,0);
        setElement(3,1,0);
        setElement(3,2,0);
        setElement(3,3,1);
    }

    inline void myMatrices::rotateZ_T(float theta)
    {
        rows = 4;
        cols = 4;
        setElement(0,0,cosf(theta));
        setElement(0,1,-sinf(theta));
        setElement(0,2,0);
        setElement(0,3,0);
        setElement(1,0,sinf(theta));
        setElement(1,1,cosf(theta));
        setElement(1,2,0);
        setElement(1,3,0);
        setElement(2,0,0);
        setElement(2,1,0);
        setElement(2,2,1);
        setElement(2,3,0);
        setElement(3,0,0);
        setElement(3,1,0);
        setElement(3,2,0);
        setElement(3,3,1);
    }

    /* 平移齐次变换矩阵 */
    inline void myMatrices::transform_T(float x,float y,float z)
    {
        rows = 4;
        cols = 4;
        setElement(0,0,1);
        setElement(0,1,0);
        setElement(0,2,0);
        setElement(0,3,x);
        setElement(1,0,0);
        setElement(1,1,1);
        setElement(1,2,0);
        setElement(1,3,y);
        setElement(2,0,0);
        setElement(2,1,0);
        setElement(2,2,1);
        setElement(2,3,z);
        setElement(3,0,0);
        setElement(3,1,0);
        setElement(3,2,0);
        setElement(3,3,1);

    }

    /* 矩阵加法 */
    inline myMatrices myMatrices::operator+(const myMatrices &other) const
    {

        myMatrices result(rows, cols);

        if (rows != other.rows || cols != other.cols)
        {
            // throw std::invalid_argument("Matrix dimensions are not compatible for addition");
        }
        else
        {
            for (uint16_t i = 0; i < rows; i++)
            {
                for (uint16_t j = 0; j < cols; j++)
                {
                    result.setElement(i, j, getElement(i, j) + other.getElement(i, j));
                }
            }
        }

        return result;
    }

    /* 矩阵减法 */
    inline myMatrices myMatrices::operator-(const myMatrices &other) const
    {

        myMatrices result(rows, cols);

        if (rows != other.rows || cols != other.cols)
        {
            // throw std::invalid_argument("Matrix dimensions are not compatible for subtraction");
            return result;
        }
        else
        {
            for (uint16_t i = 0; i < rows; i++)
            {
                for (uint16_t j = 0; j < cols; j++)
                {
                    result.setElement(i, j, getElement(i, j) - other.getElement(i, j));
                }
            }
        }

        return result;
    }

    /* 矩阵乘法 */

    inline myMatrices myMatrices::operator*(const myMatrices &other) const
    {

        myMatrices result(rows, other.cols);

        if (cols != other.rows)
        {
            // throw std::invalid_argument("Matrix dimensions are not compatible for multiplication");
        }
        else
        {
            for (uint16_t i = 0; i < rows; i++)
            {
                for (uint16_t j = 0; j < other.cols; j++)
                {
                    float sum = 0;
                    for (uint16_t k = 0; k < cols; k++)
                    {
                        sum += getElement(i, k) * other.getElement(k, j);
                    }
                    result.setElement(i, j, sum);
                }
            }
        }

        return result;
    }

    /* 矩阵与常数相乘 */

    inline myMatrices myMatrices::operator*(const float other) const
    {
        myMatrices result(rows, cols);
        for (uint16_t i = 0; i < rows; i++)
        {
            for (uint16_t j = 0; j < cols; j++)
            {
                result.setElement(i, j, this->getElement(i, j) * other);
            }
        }
        return result;
    }

    /* 矩阵转置 */

    inline myMatrices myMatrices::transpose() const
    {
        myMatrices result(cols, rows);

        for (uint16_t i = 0; i < rows; i++)
        {
            for (uint16_t j = 0; j < cols; j++)
            {
                result.setElement(j, i, getElement(i, j));
            }
        }

        return result;
    }

    /* 矩阵求逆 */

    inline myMatrices myMatrices::inverse() const
    {

        myMatrices adj(rows, cols);

        if (rows != cols)
        {
            // throw std::invalid_argument("Matrix is not square");
            return adj;
        }

        float det = determinant();
        if (det == 0)
        {
            // throw std::runtime_error("Matrix is singular; cannot compute inverse");
            return adj;
        }

        for (uint16_t i = 0; i < rows; i++)
        {
            for (uint16_t j = 0; j < cols; j++)
            {
                // 计算(i, j)位置的代数余子式
                myMatrices cofactorMatrix = getCofactorMatrix(i, j);
                double cofactor = cofactorMatrix.determinant();

                // 使用伴随矩阵的(i, j)位置存储代数余子式
                adj.setElement(i, j, cofactor);
            }
        }

        return adj * (1. / det);
    }

    /* 计算矩阵的行列式 */

    inline float myMatrices::determinant() const
    {
        if (rows != cols)
        {
            // throw std::invalid_argument("Matrix must be square to calculate determinant");
            return 0;
        }

        if (rows == 1)
        {
            return getElement(0, 0);
        }

        if (rows == 2)
        {
            // 对于2x2矩阵，行列式计算公式为 ad - bc
            return getElement(0, 0) * getElement(1, 1) - getElement(0, 1) * getElement(1, 0);
        }

        float det = 0;
        for (uint16_t i = 0; i < cols; i++)
        {
            // 计算代数余子式的值
            float cofactor = getElement(0, i) * getCofactorMatrix(0, i).determinant();
            // 使用递归计算行列式
            det += (i % 2 == 0 ? 1 : -1) * cofactor;
        }

        return det;
    }

    /* 创建代数余子式 */

    inline myMatrices myMatrices::getCofactorMatrix(uint16_t rowToRemove, uint16_t colToRemove) const
    {

        myMatrices cofactor(rows - 1, cols - 1);

        if (rowToRemove < 0 || rowToRemove >= rows || colToRemove < 0 || colToRemove >= cols)
        {
            // throw std::invalid_argument("Invalid row or column index");
            return cofactor;
        }

        uint16_t cofactorRow = 0;
        for (uint16_t i = 0; i < rows; i++)
        {
            if (i == rowToRemove)
            {
                continue; // 跳过要移除的行
            }

            uint16_t cofactorCol = 0;
            for (uint16_t j = 0; j < cols; j++)
            {
                if (j == colToRemove)
                {
                    continue; // 跳过要移除的列
                }

                cofactor.setElement(cofactorRow, cofactorCol, getElement(i, j));
                cofactorCol++;
            }

            cofactorRow++;
        }

        return cofactor;
    }

    /* 矩阵行增广 */

    inline void myMatrices::rowExpansion(myMatrices other)
    {
        if (cols != other.cols)
        {
            // throw  std::invalid_argument("Matrix dimensions are not compatible for expansion");
        }
        else
        {
            float *p = data;                                                                         //读取原有内存地址
            // data = new float[(rows + other.rows) * cols];                                            //重新分配内存
            memcpy(data, p, sizeof(float) * rows * cols);                                            //将原数据copy
            memcpy(data + (rows * cols), other.getArray(), sizeof(float) * other.rows * other.cols); // copy增广数据
            rows = rows + other.rows;                                                                //更改行数
            delete p;                                                                                //删除原数据内存
        }
    }

    /* 矩阵列增广 */

    inline void myMatrices::colExpansion(myMatrices other)
    {
        if (rows != other.rows)
        {
            // throw  std::invalid_argument("Matrix dimensions are not compatible for expansion");
        }
        else
        {
            float *p = data;                              //读取原有内存地址
            // data = new float[rows * (cols + other.cols)]; //重新分配内存
            uint16_t n = cols + other.cols;
            for (uint16_t i = 0; i < rows; i++)
            {
                memcpy(data + i * n, p + i * cols, sizeof(float) * cols);                                   //将原数据copy
                memcpy(data + i * n + cols, other.getArray() + i * other.cols, sizeof(float) * other.cols); // copy增广数据
            }
            cols = cols + other.cols; //更改列数
            delete p;                 //删除原数据内存
        }
    }

    /* 计算 Kronecker 乘积（参考matlab的kron函数） */
    template <class U>
    inline myMatrices kron(myMatrices &A, myMatrices &B)
    {
        uint16_t m = A.getRows();
        uint16_t n = A.getCols();
        uint16_t p = B.getRows();
        uint16_t q = B.getCols();

        myMatrices result(m * p, n * q);

        for (uint16_t i = 0; i < m; i++)
        {
            for (uint16_t j = 0; j < n; j++)
            {
                for (uint16_t k = 0; k < p; k++)
                {
                    for (uint16_t l = 0; l < q; l++)
                    {
                        result.setElement(i * p + k, j * q + l, A.getElement(i, j) * B.getElement(k, l));
                    }
                }
            }
        }

        return result;
    }

    template <class U, class... Args>
    inline myMatrices kron(myMatrices &A, myMatrices &B, Args &...rest)
    {
        uint16_t m = A.getRows();
        uint16_t n = A.getCols();
        uint16_t p = B.getRows();
        uint16_t q = B.getCols();

        myMatrices result(m * p, n * q);

        for (uint16_t i = 0; i < m; i++)
        {
            for (uint16_t j = 0; j < n; j++)
            {
                for (uint16_t k = 0; k < p; k++)
                {
                    for (uint16_t l = 0; l < q; l++)
                    {
                        result.setElement(i * p + k, j * q + l, A.getElement(i, j) * B.getElement(k, l));
                    }
                }
            }
        }

        return kron(result, rest...);
    }

    /* 构建对角块矩阵 */
    template <class U>
    inline myMatrices blkdiag(const myMatrices &A, const myMatrices &B)
    {

        myMatrices result(A.getRows() + B.getRows(), A.getCols() + B.getCols());

        uint16_t rowOffset = 0;
        uint16_t colOffset = 0;

        for (uint16_t j = 0; j < A.getRows(); j++)
        {
            for (uint16_t k = 0; k < A.getCols(); k++)
            {
                result.setElement(rowOffset + j, colOffset + k, A.getElement(j, k));
            }
        }

        rowOffset += A.getRows();
        colOffset += A.getCols();

        for (uint16_t j = 0; j < B.getRows(); j++)
        {
            for (uint16_t k = 0; k < B.getCols(); k++)
            {
                result.setElement(rowOffset + j, colOffset + k, B.getElement(j, k));
            }
        }

        rowOffset += B.getRows();
        colOffset += B.getCols();

        return result;
    }

    template <class U, class... Args>
    inline myMatrices blkdiag(const myMatrices &A, const myMatrices &B, const Args &...rest)
    {

        myMatrices result(A.getRows() + B.getRows(), A.getCols() + B.getCols());

        uint16_t rowOffset = 0;
        uint16_t colOffset = 0;

        for (uint16_t j = 0; j < A.getRows(); j++)
        {
            for (uint16_t k = 0; k < A.getCols(); k++)
            {
                result.setElement(rowOffset + j, colOffset + k, A.getElement(j, k));
            }
        }

        rowOffset += A.getRows();
        colOffset += A.getCols();

        for (uint16_t j = 0; j < B.getRows(); ++j)
        {
            for (uint16_t k = 0; k < B.getCols(); ++k)
            {
                result.setElement(rowOffset + j, colOffset + k, B.getElement(j, k));
            }
        }

        rowOffset += B.getRows();
        colOffset += B.getCols();

        return blkdiag(result, rest...);
    }

    /* 创建零矩阵 */
    template <class U>
    inline myMatrices zeros(uint16_t n)
    {
        myMatrices result(n, n);
        return result;
    }
    template <class U>
    inline myMatrices zeros(uint16_t rows, uint16_t cols)
    {
        myMatrices result(rows, cols);
        return result;
    }

    /* 创建单位对角阵 */
    template <class U>
    inline myMatrices eye(uint16_t n)
    {
        myMatrices result(n, n);
        for (uint16_t i = 0; i < n; i++)
        {
            result.setElement(i, i, 1);
        }
        return result;
    }

    /* 矩阵行向合并（行不变列变） */
    template <class U>
    myMatrices rowCombine(myMatrices &A, myMatrices &B)
    {
        myMatrices result(A.getRows() + B.getRows(), A.getCols());
        if (A.getCols() != B.getCols())
        {
            // throw  std::invalid_argument("Matrix dimensions are not compatible for combine");
            return result;
        }
        else
        {
            memcpy(result.data, A.getArray(), sizeof(U) * A.getRows() * A.getCols());                               //将原数据copy
            memcpy(result.data + (A.getRows() * A.getCols()), B.getArray(), sizeof(U) * B.getRows() * B.getCols()); // copy增广数据
            return result;
        }
    }

    template <class U, class... Args>
    myMatrices rowCombine(myMatrices &A, myMatrices &B, Args... rest)
    {
        myMatrices result(A.getRows() + B.getRows(), A.getCols());
        if (A.getCols() != B.getCols())
        {
            // throw  std::invalid_argument("Matrix dimensions are not compatible for combine");
            return result;
        }
        else
        {
            memcpy(result.data, A.getArray(), sizeof(U) * A.getRows() * A.getCols());                               //将原数据copy
            memcpy(result.data + (A.getRows() * A.getCols()), B.getArray(), sizeof(U) * B.getRows() * B.getCols()); // copy增广数据
            return rowCombine(result, rest...);
        }
    }

    /* 矩阵列向合并（行变列不变） */
    template <class U>
    myMatrices colCombine(myMatrices &A, myMatrices &B)
    {
        myMatrices result(A.getRows(), A.getCols() + B.getCols());
        if (A.getRows() != B.getRows())
        {
            // throw  std::invalid_argument("Matrix dimensions are not compatible for combine");
            return result;
        }
        else
        {
            uint16_t n = A.getCols() + B.getCols();
            for (uint16_t i = 0; i < A.getRows(); i++)
            {
                memcpy(result.data + i * n, A.getArray() + i * A.getCols(), sizeof(U) * A.getCols());               //将原数据copy
                memcpy(result.data + i * n + A.getCols(), B.getArray() + i * B.getCols(), sizeof(U) * B.getCols()); // copy增广数据
            }
            return result;
        }
    }

    template <class U, class... Args>
    myMatrices colCombine(myMatrices &A, myMatrices &B, Args... rest)
    {
        myMatrices result(A.getRows(), A.getCols() + B.getCols());
        if (A.getRows() != B.getRows())
        {
            // throw  std::invalid_argument("Matrix dimensions are not compatible for combine");
            return result;
        }
        else
        {
            uint16_t n = A.getCols() + B.getCols();
            for (uint16_t i = 0; i < A.getRows(); i++)
            {
                memcpy(result.data + i * n, A.getArray() + i * A.getCols(), sizeof(U) * A.getCols());               //将原数据copy
                memcpy(result.data + i * n + A.getCols(), B.getArray() + i * B.getCols(), sizeof(U) * B.getCols()); // copy增广数据
            }
            return colCombine(result, rest...);
        }
    }

}

#endif
