#pragma once
#include "svd3_cuda.h"
#include "saiga/core/math/Types.h"


inline HD void svd(const Saiga::mat3 &A, Saiga::mat3 &U, Saiga::mat3 &S, Saiga::mat3 &V)
{
        float u11, u12, u13, u21, u22, u23, u31, u32, u33;
        float s11, s12, s13, s21, s22, s23, s31, s32, s33;
        float v11, v12, v13, v21, v22, v23, v31, v32, v33;
        svd(A(0,0), A(0,1), A(0,2),
           A(1,0), A(1,1), A(1,2),
           A(2,0), A(2,1), A(2,2),
           u11, u12, u13, u21, u22, u23, u31, u32, u33,
           s11, s12, s13, s21, s22, s23, s31, s32, s33,
           v11, v12, v13, v21, v22, v23, v31, v32, v33);
        U << u11, u12, u13, u21, u22, u23, u31, u32, u33;
        S << s11, s12, s13, s21, s22, s23, s31, s32, s33;
        V << v11, v12, v13, v21, v22, v23, v31, v32, v33;
}

inline HD Saiga::mat3 pd(const Saiga::mat3 &A)
{     
    Saiga::mat3 U, S, V;
    svd(A, U, S, V);

    return U*V.transpose();
}
