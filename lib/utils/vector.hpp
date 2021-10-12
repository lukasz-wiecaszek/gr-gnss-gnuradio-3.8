/**
 * @file vector.hpp
 *
 * Taken from wiki:
 * A vector space (also called a linear space) is a set of objects called vectors,
 * which may be added together and multiplied ("scaled") by numbers, called scalars.
 * Scalars are often taken to be real numbers, but there are also vector spaces with
 * scalar multiplication by complex numbers, rational numbers, or generally any field.
 *
 * @author Lukasz Wiecaszek <lukasz.wiecaszek@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 */

#ifndef _VECTOR_HPP_
#define _VECTOR_HPP_

/*===========================================================================*\
 * system header files
\*===========================================================================*/
#include <cstddef>
#include <cmath>
#include <array>
#include <string>
#include <sstream>

/*===========================================================================*\
 * project header files
\*===========================================================================*/

/*===========================================================================*\
 * preprocessor #define constants and macros
\*===========================================================================*/

/*===========================================================================*\
 * global type definitions
\*===========================================================================*/
namespace lts
{

template<typename T, std::size_t N>
class vector
{
public:
    explicit vector() :
        m_array{}
    {
    }

    template<typename... U>
    explicit vector(U&&... args) :
        m_array{{std::forward<U>(args)...}}
    {
        static_assert(sizeof...(U) == N, "Invalid dimension of constructor");
    }

    template<std::size_t I>
    const T& get()
    {
        return std::get<I>(m_array);
    }

    std::string to_string() const
    {
        std::ostringstream stream;

        stream << "(";

        for (std::size_t i = 0; i < N; ++i) {
            stream << std::scientific << m_array[i];
            if (i < (N - 1))
                stream << ", ";
        }

        stream << ")";

        return stream.str();
    }

    operator std::string() const
    {
        return to_string();
    }

    vector& operator += (const vector& v)
    {
        for (std::size_t i = 0; i < N; ++i)
            m_array[i] += v.m_array[i];
        return *this;
    }

    vector& operator -= (const vector& v)
    {
        for (std::size_t i = 0; i < N; ++i)
            m_array[i] -= v.m_array[i];
        return *this;
    }

    vector& operator *= (const T& s)
    {
        for (std::size_t i = 0; i < N; ++i)
            m_array[i] *= s;
        return *this;
    }

    vector& operator /= (const T& s)
    {
        for (std::size_t i = 0; i < N; ++i)
            m_array[i] /= s;
        return *this;
    }

    template<typename T1, std::size_t N1>
    friend vector<T1, N1> operator + (const vector<T1, N1>& v1, const vector<T1, N1>& v2);

    template<typename T1, std::size_t N1>
    friend T1 operator * (const vector<T1, N1>& v1, const vector<T1, N1>& v2);

    template<typename T1, std::size_t N1>
    friend vector<T1, N1> operator * (const T1& s, const vector<T1, N1>& v);

    template<typename T1, std::size_t N1>
    friend vector<T1, N1> operator * (const vector<T1, N1>& v, const T1& s);

    template<typename T1, std::size_t N1>
    friend vector<T1, N1> operator / (const vector<T1, N1>& v, const T1& s);

private:
    std::array<T, N> m_array;
};

} /* end of namespace lts */

/*===========================================================================*\
 * inline function/variable definitions
\*===========================================================================*/
namespace lts
{

template<typename T1, std::size_t N1>
inline vector<T1, N1> operator + (const vector<T1, N1>& v1, const vector<T1, N1>& v2)
{
    vector<T1, N1> retval;

    for (std::size_t i = 0; i < N1; ++i)
        retval.m_array[i] = v1.m_array[i] + v2.m_array[i];

    return retval;
}

template<typename T1, std::size_t N1>
inline T1 operator * (const vector<T1, N1>& v1, const vector<T1, N1>& v2)
{
    T1 retval = 0;

    for (std::size_t i = 0; i < N1; ++i)
        retval += v1.m_array[i] * v2.m_array[i];

    return retval;
}

template<typename T1, std::size_t N1>
inline vector<T1, N1> operator * (const T1& s, const vector<T1, N1>& v)
{
    vector<T1, N1> retval;

    for (std::size_t i = 0; i < N1; ++i)
        retval.m_array[i] = s * v.m_array[i];

    return retval;
}

template<typename T1, std::size_t N1>
inline vector<T1, N1> operator * (const vector<T1, N1>& v, const T1& s)
{
    vector<T1, N1> retval;

    for (std::size_t i = 0; i < N1; ++i)
        retval.m_array[i] = v.m_array[i] * s;

    return retval;
}

template<typename T1, std::size_t N1>
inline vector<T1, N1> operator / (const vector<T1, N1>& v, const T1& s)
{
    vector<T1, N1> retval;

    for (std::size_t i = 0; i < N1; ++i)
        retval.m_array[i] = v.m_array[i] / s;

    return retval;
}

template<typename T1, std::size_t N1>
inline T1 norm(const vector<T1, N1>& v)
{
    return v * v;
}

template<typename T1, std::size_t N1>
inline T1 abs(const vector<T1, N1>& v)
{
    return std::sqrt(norm(v));
}

} /* end of namespace lts */

/*===========================================================================*\
 * global object declarations
\*===========================================================================*/
namespace lts
{

} /* end of namespace lts */

/*===========================================================================*\
 * function forward declarations
\*===========================================================================*/
namespace lts
{

} /* end of namespace lts */

#endif /* _VECTOR_HPP_ */
